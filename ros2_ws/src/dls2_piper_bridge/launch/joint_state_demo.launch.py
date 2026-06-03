import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory("dls2_piper_bridge")
    plotjuggler_layout = os.path.join(
        package_share_dir,
        "config",
        "plotjuggler_layout.xml",
    )

    return LaunchDescription(
        [
            Node(
                package="dls2_piper_bridge",
                executable="piper_hal",
                name="piper_hal",
                output="screen",
            ),
            Node(
                package="plotjuggler",
                executable="plotjuggler",
                name="plotjuggler",
                arguments=["-l", plotjuggler_layout],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub',
                    '--rate', '10',
                    '-p', '0',
                    '/arm_control_signal',
                    'dls2_interface/msg/ArmControlSignal',
                    "{desired_arm_joints_torque: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], desired_arm_gripper_torque: 0.0}",
                ],
            output='screen'
            ),
        ]
    )
