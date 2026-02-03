#!/usr/bin/env python3
"""
Publish Kinova Gen3 joint states.

"""

import time
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import JointState
from kortex_contrib_msgs.msg import BaseFeedback

from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.RouterClient import RouterClient
from kortex_api.UDPTransport import UDPTransport
from kortex_api.autogen.messages import Session_pb2
from kortex_api.SessionManager import SessionManager


class JointStatePublisher(Node):
    def __init__(
        self,
        rate: int = 100,
        ):
        super().__init__("kortex_joint_states_publisher")

        # Parameters
        self.declare_parameter("ip", "192.168.1.127")
        self.declare_parameter("username", "admin")
        self.declare_parameter("password", "admin")

        ip = self.get_parameter("ip").get_parameter_value().string_value
        username = self.get_parameter("username").get_parameter_value().string_value
        password = self.get_parameter("password").get_parameter_value().string_value

        # Communicate with Kortex_api
        self.init_kortex_api(ip, username, password)

        # Publisher(s)
        self.joint_states_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.base_feedback_pub = self.create_publisher(BaseFeedback, "/base_feedback", 10)
        self.create_timer(1 / rate, self.publish_joint_states)

    def init_kortex_api(self, ip: str, username: str, password: str):
        """UDP Session for monitoring high-frequency messages.
        """
        self.transport = UDPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)

        UDP_PORT = 10001
        SESSION_INACTIVITY_TIMEOUT = 10000      # (milliseconds)
        CONNECTION_INACTIVITY_TIMEOUT = 2000    # (milliseconds)
        self.transport.connect(ip, UDP_PORT)
        session_info = Session_pb2.CreateSessionInfo()
        session_info.username = username
        session_info.password = password
        session_info.session_inactivity_timeout = SESSION_INACTIVITY_TIMEOUT
        session_info.connection_inactivity_timeout = CONNECTION_INACTIVITY_TIMEOUT

        self.sessionManager = SessionManager(self.router)
        self.get_logger().info(f'Logging as "{username}" at "{ip}"')
        self.sessionManager.CreateSession(session_info)
        
        self.base_cyclic = BaseCyclicClient(self.router)

    def close_kortex_api(self):
        self.sessionManager.CloseSession()
        self.transport.disconnect()
        time.sleep(1.0)

    def publish_joint_states(self):
        state = JointState()
        try:
            base_feedback = self.base_cyclic.RefreshFeedback()
            state.header.stamp = self.get_clock().now().to_msg()
            state.name = [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
                "joint_7",
                "robotiq_85_left_knuckle_joint",
            ]
            state.position = [
                np.deg2rad(base_feedback.actuators[0].position),
                np.deg2rad(base_feedback.actuators[1].position),
                np.deg2rad(base_feedback.actuators[2].position),
                np.deg2rad(base_feedback.actuators[3].position),
                np.deg2rad(base_feedback.actuators[4].position),
                np.deg2rad(base_feedback.actuators[5].position),
                np.deg2rad(base_feedback.actuators[6].position),
                base_feedback.interconnect.gripper_feedback.motor[0].position / 100,
            ]
            self.joint_states_pub.publish(state)

            # Publish cartesian tool pose (in degree)
            # TODO: Publish full base feedbacks like ros_kortex
            base_state = BaseFeedback()
            base_state.tool_pose_x = base_feedback.base.tool_pose_x     # meters
            base_state.tool_pose_y = base_feedback.base.tool_pose_y
            base_state.tool_pose_z = base_feedback.base.tool_pose_z
            base_state.tool_pose_theta_x = np.deg2rad(base_feedback.base.tool_pose_theta_x)     # degrees -> radians
            base_state.tool_pose_theta_y = np.deg2rad(base_feedback.base.tool_pose_theta_y)
            base_state.tool_pose_theta_z = np.deg2rad(base_feedback.base.tool_pose_theta_z)
            self.base_feedback_pub.publish(base_state)

        except:
            self.get_logger().error("Failed to publish joint states")


def main(args=None):
    rclpy.init(args=args)

    node = JointStatePublisher()

    try:
        rclpy.spin(node)
    finally:
        node.close_kortex_api()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
