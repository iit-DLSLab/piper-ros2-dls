import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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
        ]
    )
