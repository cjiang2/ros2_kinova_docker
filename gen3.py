#!/usr/bin/env python3
"""
python3 gen3.py
--------------------

ROS 2 node that drives a Kinova Gen3 arm with ros2_kortex.

* Subscribes to: /joint_trajectory_controller/state
                 /camera/camera/color/image_rect_raw
                 /camera/camera/aligned_depth_to_color/image_raw
* Publishes to : /joint_trajectory_controller/joint_trajectory
* Runs at      : ? Hz
"""
from typing import Tuple
import time
import math
import threading
import sys
import traceback
from functools import partial

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, JointState
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand

from kortex_contrib_msgs.srv import SendJointTrajectory


class Gen3VisualEnvironment(Node):
    """ROS2 node for interacting with a Gen3 robot and camera.
    """
    # ROS topics and joint names
    STATE_TOPIC = '/joint_states'
    CMD_TOPIC = '/joint_trajectory_controller/joint_trajectory'

    # /camera/camera/color/image_rect_raw
    # /camera/camera/aligned_depth_to_color/image_raw
    # /twist_controller/commands

    # Joint names and indices
    JOINT_NAMES = [
        'joint_1',
        'joint_2',
        'joint_3',
        'joint_4',
        'joint_5',
        'joint_6',
        'joint_7',
    ]
    
    JOINT_NAME_TO_IDX = {
        'joint_1': 0,
        'joint_2': 1,
        'joint_3': 2,
        'joint_4': 3,
        'joint_5': 4,
        'joint_6': 5,
        'joint_7': 6
    }

    def __init__(
        self,
        color_topic: str = "/camera/camera/color/image_rect_raw",
        depth_topic: str = "/camera/camera/aligned_depth_to_color/image_raw",
        ):
        super().__init__('gen3_camera_node')
        self.color_topic = color_topic
        self.depth_topic = depth_topic

        self.bridge = CvBridge()
        self.joint_positions = None  # Dictionary of current joint positions
        self.img, self.depth = None, None

        # Subscriber for controller state messages
        self.create_subscription(
            JointState,
            self.STATE_TOPIC,
            self.state_callback,
            10
        )

        # Create camera subscription(s)
        if color_topic:
            self.create_subscription(Image, color_topic, self.color_callback, 10)
        if depth_topic:
            self.create_subscription(Image, depth_topic, self.depth_callback, 10)

        # Publisher for joint trajectory commands
        self.joint_traj_pub = self.create_publisher(JointTrajectory, self.CMD_TOPIC, 10)

        # Action client for gripper
        # Keep action client(s) inside their own Reentrant Callback
        # to run with subscriber(s) in parallel
        # https://discourse.openrobotics.org/t/how-to-use-callback-groups-in-ros2/25255
        # https://karelics.fi/blog/2022/04/21/deadlocks-in-rclpy/
        self.callback_group = ReentrantCallbackGroup()      
        self.gripper_cmd_client = ActionClient(
            self, GripperCommand, 
            '/robotiq_gripper_controller/gripper_cmd', 
            callback_group=self.callback_group)
        
        # self.follow_joint_traj_client = ActionClient(
        #     self, FollowJointTrajectory, 
        #     '/joint_trajectory_controller/follow_joint_trajectory',
        #     callback_group=self.callback_group
        # )
        self.send_traj_client = self.create_client(
            SendJointTrajectory,
            "/send_joint_trajectory",
            callback_group=self.callback_group,
        )
        
        self.get_logger().info("Gen3 node initialized.")

    def state_callback(self, msg: JointState):
        """Callback for receiving controller state messages (joint positions and velocities).
        """
        # Subscribed to: /joint_states
        # NOTE: ros2_kortex publishes in the wrong order: 
        # ['joint_1', 'robotiq_85_left_knuckle_joint', 'joint_2', 'joint_4', 'joint_5', 'joint_3', 'joint_6', 'joint_7']
        self.joint_positions = [msg.position[msg.name.index(f"joint_{i}")] for i in range(1, len(self.JOINT_NAMES) + 1)]
        
        # Subscribed to: /joint_trajectory_controller/controller_state
        # self.joint_positions = msg.actual.positions[:len(self.JOINT_NAMES)]
        # self.joint_velocities = msg.actual.velocities[:len(self.JOINT_NAMES)]

    def color_callback(self, msg: Image):
        self.img = self.bridge.imgmsg_to_cv2(msg, "rgb8")

    def depth_callback(self, msg: Image):
        self.depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')


    # -----
    # Publisher(s)
    # -----
    def send_joint_angles_async(self, goal: Tuple[float], duration: float = 0.0):
        """Send a non-blocking, delta joint position (rad) to the robot.
        """
        if isinstance(goal, np.ndarray):
            goal = goal.tolist()

        if len(goal) != 7:
            raise Exception(f"Expected 7 joint positions, got {len(goal)}!")
            
        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = goal
        sec, nanosec = math.modf(duration)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)  # Temps pour atteindre la position

        # Publish once only
        traj.points.append(point)
        self.joint_traj_pub.publish(traj)


    # -----
    # Action clinet(s)
    # -----
    def _wait_future(self, future, timeout: float = 20.0):
        """Safely wait for an rclpy future using a threading.Event.
        """
        event = threading.Event()
        def _done(fut):
            event.set()
        future.add_done_callback(_done)

        # Wait for future
        # NOTE: Slight chance of false timeouts at the boundary
        while not future.done():
            time.sleep(0.1)     
            if not event.wait(timeout=timeout):
                self.get_logger().error(f"Timeout after ({timeout}s).")
                return None, False

        # Future is done, propagate any exception
        try:
            return future.result(), True
        except Exception as e:
            self.get_logger().error("{}".format(e))
            return None, False


    def send_gripper_command(self, position: float, max_effort: float = 100.0):
        """Send a blocking gripper command.
        """
        if not self.gripper_cmd_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("gripper_cmd server is not available!")
            return False
        
        cmd = GripperCommand.Goal()
        cmd.command.position = position
        cmd.command.max_effort = max_effort

        # -----
        # Send goal and wait to be accepted/rejected
        self.get_logger().info("Sending gripper cmd...")
        send_goal_future = self.gripper_cmd_client.send_goal_async(cmd)
        goal_handle, ok = self._wait_future(send_goal_future)
        if not ok or not goal_handle.accepted:
            self.get_logger().error("Gripper cmd rejected!")
            return False
        
        # Block until gripper command is completed
        cmd_future = goal_handle.get_result_async()
        result, ok = self._wait_future(cmd_future)
        if not ok: 
            self.get_logger().error("Gripper cmd failed!")
            return False

        self.get_logger().info('Gripper finished: {}'.format(result.result))
        return True
    
    
    # -----
    # Service clinet(s)
    # -----
    def send_joint_angles(self, goal: Tuple[float]):
        """Send a joint trajectory (in rads) and blocks until the robot arrives.
        """
        if isinstance(goal, np.ndarray):
            goal = goal.tolist()
        if len(goal) != 7:
            raise Exception(f"Expected 7 joint positions, got {len(goal)}!")

        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [float(rad) for rad in goal]
        point.time_from_start = Duration(sec=0, nanosec=0)
        traj.points.append(point)

        req = SendJointTrajectory.Request()
        req.traj = traj

        # Block till future completes
        fut = self.send_traj_client.call_async(req)
        result, ok = self._wait_future(fut)
        if not ok or result is None or not result.ok:
            self.get_logger().error(f"Service call failed: {result}")
            return False
        return True


def exception_hook_with_shutdown(thread, node):
    def hook(exc_type, exc_value, exc_traceback):
        """Global handler to shutdown in case of exception.
        """
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        node.destroy_node()
        rclpy.shutdown()
        thread.join()
        sys.exit(1)
    return hook

def send_gen3_to_test(env):
    """Gen3 will point forward.
    """
    goal = [
        -0.0037103160306983796,
        -0.37204786534181356,
        -3.1326697323574826,
        -2.3142302619476394,
        0.0007629473435017953,
        -0.2223744836538346,
        1.5677635189455141,
    ]
    env.send_joint_angles(goal)
    env.send_gripper_command(0.0)


def send_gen3_home(env):
    goal_angle = [
        -0.1336059570312672, 
        -28.57940673828129, 
        -179.4915313720703, 
        -147.7, 
        0.06742369383573531, 
        -57.420898437500036, 
        89.88030242919922,
    ]
    position = np.deg2rad(np.array(goal_angle))
    env.send_joint_angles(position)


def main(args=None):
    rclpy.init(args=args)

    env = Gen3VisualEnvironment()

    # Spawn an executor to allow callbacks to run in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(env)

    # Create a separate thread to run the executor's spin method
    # The 'spin' method is a blocking call that processes incoming messages.
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    time.sleep(1.5)

    # Register a global exception for safe shutdown
    sys.excepthook = exception_hook_with_shutdown(executor_thread, env)

    time.sleep(3.0)
    send_gen3_to_test(env, env)

    # # -----
    # # Main thread is free to do other things
    # # -----
    # # Some extra test(s)
    # send_gen3_to_test(env)
    # time.sleep(1.0)
    # print("Current:", env.joint_positions)

    # # This will spin the 7th joint a bit
    # i = 0
    # gripper_pos = 0.0
    # goal = np.array(env.joint_positions).copy()
    # orig = goal.copy()
    # print(goal)

    # while True:
    #     i += 1
    #     goal[6] += 0.15
    #     gripper_pos += 0.05
    #     print(env.joint_positions, "Goal:", goal)
    #     env.send_joint_angles(goal)
    #     env.send_gripper_command(gripper_pos)

    #     # plt.subplot(1,2,1)
    #     # plt.imshow(env.img)
    #     # plt.subplot(1,2,2)
    #     # plt.imshow(env.depth)
    #     # plt.pause(0.001)

    #     if input("Press enter to continue: ") or i > 3:
    #         break
    #     # plt.clf()

    # env.send_joint_angles(orig)

    # # This will spin the gripper a bit
    # if not input("Enter to send gripper command: "):
    #     env.send_gripper_command(0.5)
    #     print("done.")
    #     env.send_gripper_command(0.0)

    # if not input("Enter to send to home: "):
    #     env.send_gripper_command(0.0)
    #     send_gen3_home(env)

    # Clean up
    env.get_logger().info('Shutting down...')
    executor.shutdown()
    env.destroy_node()
    rclpy.shutdown()
    executor_thread.join() # Ensure the executor thread finishes

if __name__ == '__main__':
    main()