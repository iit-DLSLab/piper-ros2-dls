import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('agx_arm_compliance_bridge'),
        'config',
        'compliance_bridge.yaml',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_config,
        description='Path to the YAML file with compliance bridge parameters.',
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace to push the node (and its topics) under.',
    )

    node = Node(
        package='agx_arm_compliance_bridge',
        executable='compliance_bridge_node',
        name='agx_arm_compliance_bridge',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([params_file_arg, namespace_arg, node])
