from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    declared_arguments = []

    # Gen3 initial arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ip",
            description="IP address by which the robot can be reached.",
            default_value="192.168.1.127",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "username",
            description="Robot session username.",
            default_value="admin",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "password",
            description="Robot session password.",
            default_value="admin",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            default_value="robotiq_2f_85",
            description="Name of the gripper attached to the arm",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "dof",
            default_value="7",
            description="DoF of robot.",
        )
    )

    # robot_state_publisher
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="kortex_description",
            description=(
                "Description package with robot URDF/XACRO files. "
                "Usually the argument is not set; it enables use of "
                "a custom description."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="gen3.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    # Initialize arguments
    ip = LaunchConfiguration("ip")
    dof = LaunchConfiguration("dof")
    gripper = LaunchConfiguration("gripper")
    username = LaunchConfiguration("username")
    password = LaunchConfiguration("password")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    # Joint states
    joint_states_publisher = Node(
        package="kortex_contrib",
        executable="joint_state_publisher",
        output="screen",
        parameters=[
            {
                "ip": ip,
                "dof": dof,
                "username": username,
                "password": password,
            }
        ],
    )

    # Robot state publisher / TF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(description_package),
                    "robots",
                    description_file,
                ]
            ),
            " ",
            "arm:=",
            "gen3",
            " ",
            "robot_ip:=",
            ip,
            " ",
            "dof:=",
            dof,
            " ",
            "gripper:=",
            gripper,
        ]
    )
    robot_description = {
        "robot_description": robot_description_content,
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Kortex TCP/tool frame.
    #
    # The Kinova Web App is configured with:
    #   x = 0 m
    #   y = 0 m
    #   z = 0.12 m
    #   theta_x = theta_y = theta_z = 0
    #
    # This fixed transform makes the ROS frame `kortex_tool_frame`
    # coincide with the TCP used by Kortex Cartesian feedback/control:
    #
    #   T_end_effector_kortex_tool =
    #       [[1, 0, 0, 0   ],
    #        [0, 1, 0, 0   ],
    #        [0, 0, 1, 0.12],
    #        [0, 0, 0, 1   ]]
    #
    # It compensates for the missing/non-equivalent `tool_frame`
    # in the current Kortex + Robotiq URDF description.
    kortex_tool_frame_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="kortex_tool_frame_publisher",
        output="screen",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0.12",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "end_effector_link",
            "--child-frame-id", "kortex_tool_frame",
        ],
    )

    # High-level controller
    high_level_control_node = Node(
        package="kortex_contrib",
        executable="high_level_movement",
        output="screen",
        parameters=[
            {
                "ip": ip,
                "dof": dof,
                "username": username,
                "password": password,
            }
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            joint_states_publisher,
            high_level_control_node,
            robot_state_publisher_node,
            kortex_tool_frame_publisher,
        ]
    )
