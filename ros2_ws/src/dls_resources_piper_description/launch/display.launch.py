from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            default_value="true",
            choices=["true", "false"],
            description="Whether to include the Piper gripper.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Prefix for the tf names.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_state_publisher_frequency",
            default_value="1000.0",
            description="The frequency in Hz for robot_state_publisher.",
        )
    )

    description_package = FindPackageShare("dls_resources_piper_description")
    description_file = PathJoinSubstitution(
        [description_package, "urdf", "piper_description.xacro"]
    )
    rvizconfig_file = PathJoinSubstitution(
        [description_package, "rviz", "display.rviz"]
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                description_file,
                " gripper:=",
                LaunchConfiguration("gripper"),
                " tf_prefix:=",
                LaunchConfiguration("tf_prefix"),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {
                "publish_frequency": LaunchConfiguration(
                    "robot_state_publisher_frequency"
                )
            },
        ],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            {"update_rate": 200},
        ],
        output="screen",
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizconfig_file],
    )

    return LaunchDescription(
        declared_arguments
        + [
            ros2_control_node,
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
