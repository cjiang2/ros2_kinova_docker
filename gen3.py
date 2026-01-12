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
from typing import Callable, Tuple
import time

import matplotlib.pyplot as plt
import numpy as np
import rclpy
import threading
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand, FollowJointTrajectory


class Gen3VisualEnvironment(Node):
    """ROS2 node for interacting with a Gen3 robot and camera.
    """
    # ROS topics and joint names
    STATE_TOPIC = '/joint_trajectory_controller/controller_state'
    CMD_TOPIC = '/joint_trajectory_controller/joint_trajectory'

    """
    ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
        joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
        points: [
            { positions: [0, 0, 0, 0, 0, 0, 0], time_from_start: { sec: 10 } },
        ]
    }" -1
    ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.0, max_effort: 100.0}}"

    ros2 launch kortex_bringup gen3.launch.py robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true gripper:="robotiq_2f_85"

    ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
    ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true

    """

    # /camera/camera/color/image_rect_raw
    # /camera/camera/aligned_depth_to_color/image_raw


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
        self.joint_velocities = None   # List of target joint positions
        self.img, self.depth = None, None

        # Subscriber for controller state messages
        self.create_subscription(
            JointTrajectoryControllerState,
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
        # Thread-safe: Keep action client(s) inside their own Reentrant Callback
        # https://discourse.openrobotics.org/t/how-to-use-callback-groups-in-ros2/25255
        # https://karelics.fi/blog/2022/04/21/deadlocks-in-rclpy/
        self.callback_group = ReentrantCallbackGroup()      
        self.gripper_cmd_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd', callback_group=self.callback_group)
        self.follow_joint_traj_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/joint_trajectory_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        self.get_logger().info("Gen3 node initialized.")

    def state_callback(self, msg: JointTrajectoryControllerState):
        """Callback for receiving controller state messages (joint positions and velocities).
        """
        self.joint_positions = msg.actual.positions[:len(self.JOINT_NAMES)]
        self.joint_velocities = msg.actual.velocities[:len(self.JOINT_NAMES)]

    def color_callback(self, msg: Image):
        self.img = self.bridge.imgmsg_to_cv2(msg, "rgb8")

    def depth_callback(self, msg: Image):
        self.depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')


    # -----
    # Publisher(s)
    def send_joint_angles(self, goal: Tuple[float], sec: int = 1, nanosec: int = 0):
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
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)  # Temps pour atteindre la position

        # Publish once only
        traj.points.append(point)
        self.joint_traj_pub.publish(traj)


    # -----
    # Action clinet(s)
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
        while not send_goal_future.done():
            time.sleep(0.1)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper cmd rejected!")
            return False
        
        # Block until gripper command is completed
        self.get_logger().info("Goal accepted. Waiting for gripper to finish moving...")
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)

        result = result_future.result()
        self.get_logger().info('Gripper finished: {}'.format(result.result))
        return True
    
    def send_joint_angles_blocking(self, goal: Tuple[float], sec: int = 1, nanosec: int = 0):
        """Send a joint trajectory (in rads) and blocks until the robot arrives.
        """
        if not self.follow_joint_traj_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("follow_joint_trajectory server is not available!")
            return False
        
        if isinstance(goal, np.ndarray):
            goal = goal.tolist()

        if len(goal) != 7:
            raise Exception(f"Expected 7 joint positions, got {len(goal)}!")

        trajectory_goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES
        
        point = JointTrajectoryPoint()
        point.positions = goal
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)  # Temps pour atteindre la position
        
        traj.points.append(point)
        trajectory_goal.trajectory = traj

        # -----
        # Send goal and wait to be accepted/rejected
        self.get_logger().info("Sending trajectory goal...")
        send_goal_future = self.follow_joint_traj_client.send_goal_async(trajectory_goal)
        while not send_goal_future.done():
            time.sleep(0.1)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected by controller!")
            return False

        # Wait for joint(s) to reach the position
        self.get_logger().info("Goal accepted. Waiting for robot to finish moving...")
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)

        result = result_future.result()
        self.get_logger().info(f"Trajectory finished with status: {result.status}")
        return True


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
    env.send_joint_angles_blocking(position, sec=5)


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

    # # -----
    # # Main thread is free to do other things
    # i = 0
    # goal = [0.0] * 7
    # gripper_pos = 0.0

    # while True:
    #     i += 1
    #     goal[0] += 0.1
    #     gripper_pos += 0.05
    #     print(env.joint_positions, "Goal:", goal)
    #     env.send_joint_angles(goal, sec=5)
    #     env.send_gripper_command(gripper_pos)

    #     # plt.subplot(1,2,1)
    #     # plt.imshow(env.img)
    #     # plt.subplot(1,2,2)
    #     # plt.imshow(env.depth)
    #     # plt.pause(0.001)

    #     if input("Press enter to continue: ") or i > 5:
    #         break
    #     plt.clf()
    if not input("Enter to send to home: "):
        send_gen3_home(env)

    # Clean up
    env.get_logger().info('Shutting down...')
    executor.shutdown()
    env.destroy_node()
    rclpy.shutdown()
    executor_thread.join() # Ensure the executor thread finishes

if __name__ == '__main__':
    main()