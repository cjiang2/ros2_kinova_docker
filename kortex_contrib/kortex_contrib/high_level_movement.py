#!/usr/bin/env python3
"""
TCP Transport Potocol for high-level robot control, configuration.
"""
from typing import Tuple
import time
import threading

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from tf_transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand

from kortex_contrib_msgs.srv import SendJointTrajectory, SendCartesianPose

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2
from kortex_api.RouterClient import RouterClient
from kortex_api.TCPTransport import TCPTransport
from kortex_api.autogen.messages import Session_pb2
from kortex_api.SessionManager import SessionManager
from kortex_api.Exceptions import KServerException

class HighLevelMovement(Node):
    def __init__(self):
        super().__init__("kortex_high_level_movement")

        # Parameters
        self.declare_parameter("ip", "192.168.1.127")
        self.declare_parameter("username", "admin")
        self.declare_parameter("password", "admin")
        self.declare_parameter("dof", 7)

        ip = self.get_parameter("ip").get_parameter_value().string_value
        username = self.get_parameter("username").get_parameter_value().string_value
        password = self.get_parameter("password").get_parameter_value().string_value
        self.dof = self.get_parameter("dof").value

        # Communicate with Kortex_api
        self.init_kortex_api(ip, username, password)
        self.set_to_single_servoing_mode()      # Enforce single-level servoing

        # # Callback groups for handling prioritized cmds
        self.safety_group = ReentrantCallbackGroup()
        self.vel_group = MutuallyExclusiveCallbackGroup()
        self.srv_group = MutuallyExclusiveCallbackGroup()

        # Subscriber(s)
        # -----
        # SendJointSpeeds
        self.joint_vel_cmd = self.create_subscription(
            JointJog,
            "/joint_vel_cmd",
            self.joint_vel_callback,
            1,
            callback_group=self.vel_group,
        )
        
        # SendTwist
        self.twist_cmd = self.create_subscription(
            TwistStamped, 
            "/twist_controller/commands", 
            self.twist_cmd_callback, 
            1,
            callback_group=self.vel_group,
        )

        # Single-level servoing publishing
        self.joint_traj = self.create_subscription(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            self.joint_traj_callback,
            1,
            callback_group=self.vel_group,
        )

        # Safety topic(s)
        # -----
        # Stop
        self.stop_cmd = self.create_subscription(
            Empty,
            "/stop",
            self.stop_cmd_callback,
            1,
            callback_group=self.safety_group,
        )

        # Clear faults
        self.clear_faults_cmd = self.create_subscription(
            Empty,
            "/clear_faults",
            self.clear_faults_callback,
            1,
            callback_group=self.safety_group,
        )

        # Action Server(s)
        self.gripper_cmd_svr = ActionServer(    # Non-blocking
            self,
            GripperCommand,
            '/robotiq_gripper_controller/gripper_cmd',
            execute_callback=self.execute_gripper_cmd,
            callback_group=self.srv_group,
        )

        # Service(s)
        self.send_joint_angles_srv = self.create_service(
            SendJointTrajectory,
            "/send_joint_trajectory",
            self.joint_traj_srv_callback,
            callback_group=self.srv_group,
        )

        self.send_cartesian_pose_srv = self.create_service(
            SendCartesianPose,
            "/send_cartesian_pose",
            self.cartesian_move_srv_callback,
            callback_group=self.srv_group,
        )


    def init_kortex_api(self, ip: str, username: str, password: str):
        """TCP Session for sending low-frequency commands to the robot.
        """
        TCP_PORT = 10000
        SESSION_INACTIVITY_TIMEOUT = 10000      # (milliseconds)
        CONNECTION_INACTIVITY_TIMEOUT = 2000    # (milliseconds)

        self.transport = TCPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)

        self.transport.connect(ip, TCP_PORT)
        session_info = Session_pb2.CreateSessionInfo()
        session_info.username = username
        session_info.password = password
        session_info.session_inactivity_timeout = SESSION_INACTIVITY_TIMEOUT
        session_info.connection_inactivity_timeout = CONNECTION_INACTIVITY_TIMEOUT

        self.sessionManager = SessionManager(self.router)
        self.get_logger().info(f'Logging as "{username}" at "{ip}"')
        self.sessionManager.CreateSession(session_info)
        
        self.base = BaseClient(self.router)
        if self.base.GetArmState().active_state == Base_pb2.ARMSTATE_IN_FAULT:
            self.base.ClearFaults()
            time.sleep(1.0)

    def close_kortex_api(self):
        self.sessionManager.CloseSession()
        self.transport.disconnect()
        time.sleep(1.0)

    # Create closure to set an event after an END or an ABORT
    @staticmethod
    def check_for_end_or_abort(e):
        """Return a closure checking for END or ABORT notifications

        Arguments:
        e -- event to signal when the action is completed
            (will be set when an END or ABORT occurs)
        """
        def check(notification, e = e):
            print("EVENT : " + \
                Base_pb2.ActionEvent.Name(notification.action_event))
            if notification.action_event == Base_pb2.ACTION_END \
            or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()
        return check
    
    # -----
    # Movement Command(s)
    # -----
    def set_to_single_servoing_mode(self):
        """For high-level movements, use single-level servoing mode.
        """
        arm_mode = Base_pb2.ServoingModeInformation()
        print(arm_mode.servoing_mode, Base_pb2.SINGLE_LEVEL_SERVOING, Base_pb2.LOW_LEVEL_SERVOING)
        arm_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(arm_mode)
        self.get_logger().info(f'Arm Mode: Single-level Servoing')

    def joint_vel_callback(self, msg: JointJog):
        vels = msg.velocities

        # NOTE: Joint Speed duration is not supported in kortex_api
        # duration = msg.duration
        joint_speeds = Base_pb2.JointSpeeds()           # kortex_api needs degrees per second
        try:
            assert self.dof == len(vels)
            for i, vel in enumerate(vels):
                joint_speed = joint_speeds.joint_speeds.add()
                joint_speed.joint_identifier = i 
                joint_speed.value = np.rad2deg(vel)     # Convert from rad/s to deg/s
                # joint_speed.duration = duration

            self.base.SendJointSpeedsCommand(joint_speeds)

        except Exception as e:
            self.get_logger().error(f'Invalid joint_speed cmd: {joint_speeds}, error {e}')

    def twist_cmd_callback(self, msg: TwistStamped):
        cmd = Base_pb2.TwistCommand()
        cmd.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
        # cmd.reference_frame = Base_pb2.CARTESIAN_JOYSTICK
        cmd.duration = 0

        try:
            twist = cmd.twist
            twist.linear_x = msg.twist.linear.x
            twist.linear_y = msg.twist.linear.y
            twist.linear_z = msg.twist.linear.z

            twist.angular_x = np.rad2deg(msg.twist.angular.x)
            twist.angular_y = np.rad2deg(msg.twist.angular.y)
            twist.angular_z = np.rad2deg(msg.twist.angular.z)

            self.base.SendTwistCommand(cmd)

        except Exception as e:
            self.get_logger().error(f'Invalid twist_cmd: {msg.twist}, error {e}')

    # -----
    # Safety command(s)
    # -----
    def stop_cmd_callback(self, msg: Empty):
        # Simply stop the robot
        self.get_logger().info(f'Stopping the robot...')
        self.base.Stop()

    def clear_faults_callback(self, msg: Empty):
        self.get_logger().info(f'ClearFaults is invoked...')
        self.base.ClearFaults()


    # -----
    # Gripper Control
    # -----

    def execute_gripper_cmd(self, goal_handle) -> GripperCommand.Result:
        cmd = goal_handle.request.command
        position = float(cmd.position)
        # max_effort = float(cmd.max_effort)        # TODO: How does effort translate to kortex?

        # Check if position is in [0, 1]
        try:
            assert 0 <= position <= 1
        except:
            self.get_logger().error(f"Invalid gripper position: {position}")
            goal_handle.abort()
            return GripperCommand.Result(
                position=position, effort=0.0,
                stalled=True, 
                reached_goal=False,
            )
        
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger.finger_identifier = 1
        finger.value = position

        try:
            self.base.SendGripperCommand(gripper_command)
        except Exception as e:
            self.get_logger().error(f"Failed SendGripperCommand: {e}")

        # Immediate success
        goal_handle.succeed()
        return GripperCommand.Result(
            position=position, effort=0.0,
            stalled=False,
            reached_goal=True,
        )
    

    # -----
    # Joint Position Control
    # -----

    def create_angular_waypoint_list(
        self, 
        points: Tuple[Tuple[float]],
        duration: Tuple[float],
        ):
        traj = Base_pb2.WaypointList()
        traj.use_optimal_blending = False
        # traj.duration = 0.0        # Global duration is disregarded for angular waypoints

        # Assume a sequence of waypoints are provided
        for i, point in enumerate(points):
            waypoint = traj.waypoints.add()
            waypoint.name = f"p_{i}"

            point = [np.rad2deg(rad) for rad in point]
            self.get_logger().info("Debugging: {}".format(point))

            aw = Base_pb2.AngularWaypoint()
            aw.angles.extend(point)
            # Duration needs to be properly set per trajectory point
            # kortex_api will reject waypoints with invalid duration
            aw.duration = duration[i] 

            waypoint.angular_waypoint.CopyFrom(aw)

        # Validate waypoint(s)
        result = self.base.ValidateWaypointList(traj)
        if len(result.trajectory_error_report.trajectory_error_elements) != 0:
            self.get_logger().info(f'Invalid trajectory: {result.trajectory_error_report}')
            return None

        return traj
    
    def create_single_angular_waypoint(
        self, 
        point: Tuple[float],
        duration: float, 
        MAX_ANGULAR_DURATION: float = 30.0,
        ):
        point = [np.rad2deg(rad) for rad in point]
        self.get_logger().info(f"Waypoint {point}, duration: {duration}")

        # Compose a waypoint
        traj = Base_pb2.WaypointList()
        traj.use_optimal_blending = False

        waypoint = traj.waypoints.add()
        waypoint.angular_waypoint.angles.extend(point)
        waypoint.angular_waypoint.duration = duration

        # TODO: Better strategy to determine a minimal duration for moving joint angles
        # kortex uses this trial and error strategy
        try:
            result = self.base.ValidateWaypointList(traj)
            error_number = len(result.trajectory_error_report.trajectory_error_elements)
            while (error_number >= 1 and duration < MAX_ANGULAR_DURATION):
                duration += 1
                traj.waypoints[0].angular_waypoint.duration = duration
                result = self.base.ValidateWaypointList(traj)
                error_number = len(result.trajectory_error_report.trajectory_error_elements)

            self.get_logger().info("Final duration: {}".format(duration))

        except Exception as e:
            self.get_logger().info(f"Failed to validate traj: {e}")

        return traj
    
    @staticmethod
    def parse_traj_points(msg: JointTrajectory):
        """Parse from msg.
        """
        def duration_to_seconds(d: Duration) -> float:
            """Convert builtin_interfaces/Duration to float seconds.
            """
            return float(d.sec) + float(d.nanosec) * 1e-9
        
        # time_from_start to duration / waypoint
        times_sec = [duration_to_seconds(p.time_from_start) for p in msg.points]
        durations = []
        prev = 0.0
        for t in times_sec:
            durations.append(t - prev)
            prev = t
        points = [p.positions.tolist() for p in msg.points]
        return points, durations
    

    def joint_traj_callback(self, msg: JointTrajectory):
        """Non-blocking, publish to "/joint_trajectory_controller/joint_trajectory"
        NOTE: If a new goal is received while prev motion is executing, Gen3 stops
        and rejects the new goal for TCPSession.
        """
        points, durations = self.parse_traj_points(msg)
        self.get_logger().info(f"Waypoints {points}, durations: {durations}")

        # Validate
        if len(points) == 1:
            waypoints = self.create_single_angular_waypoint(points[0], durations[0])
        else:
            # TODO: How to better handle multiple waypoints?
            self.get_logger().warn(f"multi-waypoints may be unstable!!!")
            waypoints = self.create_angular_waypoint_list(points, durations)
        
        # Non-blocking call
        if waypoints is not None:
            try:
                self.base.ExecuteWaypointTrajectory(waypoints)
            except Exception as e:
                self.get_logger().error(f"Failed to execute: {e}")

    
    def joint_traj_srv_callback(self, request, response):
        """Blocking, gen3 executes the traj until completed.
        """
        traj = request.traj
        self.get_logger().info("Points: {}".format(traj))

        # Parse one point
        points, durations = self.parse_traj_points(traj)
        self.get_logger().info(f"Waypoints {points}, durations: {durations}")

        # Validate
        if len(points) == 1:
            waypoints = self.create_single_angular_waypoint(points[0], durations[0])
        else:
            # TODO: How to better handle multiple waypoints?
            self.get_logger().warn(f"multi-waypoints may be unstable!!!")
            waypoints = self.create_angular_waypoint_list(points, durations)
        
        # Blocking trajectory execution
        # Register notification handle
        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )
        self.base.ExecuteWaypointTrajectory(waypoints)
        finished = e.wait(timeout=10.0)
        self.base.Unsubscribe(notification_handle)

        if finished:
            self.get_logger().info(f"SendJointTrajectory finished")
            response.ok = True
        else:
            self.get_logger().error(f"SendJointTrajectory timeout after {10.0}s")
        
        return response
    
    # -----
    # Cartesian Pose Control
    # -----

    @staticmethod
    def _pose_to_transform(
        xyz: np.ndarray, 
        theta_xyz: np.ndarray, 
        degrees: bool = False,
        ):
        """Build an SE(3) transform from xyz translation and 
        extrinsic XYZ Tait-Bryan Euler angles.
        [[ R00 R01 R02  x ]
         [ R10 R11 R12  y ]
         [ R20 R21 R22  z ]
         [  0   0   0   1 ]]
        """
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler(
            "xyz", theta_xyz, degrees=degrees
        ).as_matrix()
        transform[:3, 3] = xyz
        return transform

    @staticmethod
    def _transform_to_kortex_pose(transform: np.ndarray):
        """Convert an SE(3) transform to back to 
        Kortex's Cartesian pose in (m, deg)."""
        xyz = transform[:3, 3]
        theta_xyz = Rotation.from_matrix(
            transform[:3, :3]
        ).as_euler("xyz", degrees=True)
        return xyz, theta_xyz


    def cartesian_move_srv_callback(self, request, response):
        """Send one cartesian (x, y, z, theta_x, theta_y, theta_z) end-effector pose.
        Position: (x, y, z), in meters.
        Orientation: (theta_x, theta_y, theta_z), in radians. (kortex accepts degrees)
        """
        pose, relative = request.pose, request.relative
        self.get_logger().info(f"Pose: {pose}, relative: {request.relative}")

        # Grab translation & rotation
        target_xyz = np.array([pose.x, pose.y, pose.z], dtype=float)
        target_theta = np.array([pose.theta_x, pose.theta_y, pose.theta_z], dtype=float)

        # Resolve relative pose
        if relative:
            # Grab the current tool pose
            current = self.base.GetMeasuredCartesianPose()
            current_transform = self._pose_to_transform(
                np.array([current.x, current.y, current.z], dtype=float),
                np.array([current.theta_x, current.theta_y, current.theta_z], dtype=float),
                degrees=True,       # kortex returns degrees by default
            )

            # Composition update: T^base_target = T^base_tool @ T_tool^target
            delta = self._pose_to_transform(target_xyz, target_theta, degrees=False)
            target_transform = current_transform @ delta
            target_xyz, target_theta_deg = self._transform_to_kortex_pose(target_transform)

        else:
            target_theta_deg = np.rad2deg(target_theta)

        # Send in kortex pose
        action = Base_pb2.Action()
        action.reach_pose.target_pose.x = float(target_xyz[0])
        action.reach_pose.target_pose.y = float(target_xyz[1])
        action.reach_pose.target_pose.z = float(target_xyz[2])
        action.reach_pose.target_pose.theta_x = float(target_theta_deg[0])
        action.reach_pose.target_pose.theta_y = float(target_theta_deg[1])
        action.reach_pose.target_pose.theta_z = float(target_theta_deg[2])

        # Blocking pose execution
        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )
        self.base.ExecuteAction(action)
        finished = e.wait(timeout=10.0)
        self.base.Unsubscribe(notification_handle)

        if finished:
            self.get_logger().info(f"SendCartesianPose finished")
            response.ok = True
        else:
            self.get_logger().error(f"SendCartesianPose timeout after {10.0}s")
        
        return response



def main(args=None):
    rclpy.init(args=args)
    node = HighLevelMovement()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.close_kortex_api()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
