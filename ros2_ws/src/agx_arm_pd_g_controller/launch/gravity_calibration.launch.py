"""Gravity torque calibration.

!!! THE ROBOT MOVES through its full range of motion. Clear the area first !!!

Produces (default locations, override with samples_output/calibration_output):
  ~/.ros/agx_arm_pd_g_controller/gravity_samples.npz
  ~/.ros/agx_arm_pd_g_controller/gravity_calibration.yaml

The calibration YAML is then passed to pd_g_controller.launch.py via the
calibration_file argument.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_OUTPUT_DIR = os.path.expanduser('~/.ros/agx_arm_pd_g_controller')


def resolve_model_path(context) -> str:
    """Model file precedence: explicit model_file arg > robot_model/use_gripper.

    agx_arm_description lays models out as
    agx_arm_urdf/<robot_model>/urdf/<robot_model>_description.urdf and
    <robot_model>_with_gripper_description.xacro for every robot (piper,
    piper_l, piper_h, piper_x, nero, ...).
    """
    model_file = context.launch_configurations['model_file']
    if model_file:
        return model_file

    robot_model = context.launch_configurations['robot_model']
    use_gripper = context.launch_configurations['use_gripper'].lower() in ('true', '1')
    filename = (
        f'{robot_model}_with_gripper_description.xacro'
        if use_gripper
        else f'{robot_model}_description.urdf'
    )
    return os.path.join(
        get_package_share_directory('agx_arm_description'),
        'agx_arm_urdf', robot_model, 'urdf', filename,
    )


def launch_setup(context):
    node = Node(
        package='agx_arm_pd_g_controller',
        executable='gravity_calibration_node',
        name='gravity_calibration',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'model_path': resolve_model_path(context),
            'num_samples': int(context.launch_configurations['num_samples']),
            'samples_output': context.launch_configurations['samples_output'],
            'calibration_output': context.launch_configurations['calibration_output'],
            'fit_model_type': context.launch_configurations['fit_model_type'],
            'start_delay': float(context.launch_configurations['start_delay']),
            'check_ground': context.launch_configurations['check_ground'].lower()
                in ('true', '1'),
            'ground_height': float(context.launch_configurations['ground_height']),
        }],
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace of the arm driver (same as the controller).',
        ),
        DeclareLaunchArgument(
            'robot_model',
            default_value='piper_l',
            description='Robot model from agx_arm_description (piper, piper_l, '
                        'piper_h, piper_x, nero, ...).',
        ),
        DeclareLaunchArgument(
            'use_gripper',
            default_value='false',
            description='Use the robot model variant with gripper.',
        ),
        DeclareLaunchArgument(
            'model_file',
            default_value='',
            description='Explicit robot model file (.xml MJCF, .urdf, or .xacro). '
                        'Overrides use_gripper.',
        ),
        DeclareLaunchArgument(
            'num_samples',
            default_value='50',
            description='Number of collision-free target configurations to visit.',
        ),
        DeclareLaunchArgument(
            'samples_output',
            default_value=os.path.join(DEFAULT_OUTPUT_DIR, 'gravity_samples.npz'),
            description='Output path for the raw samples (.npz).',
        ),
        DeclareLaunchArgument(
            'calibration_output',
            default_value=os.path.join(DEFAULT_OUTPUT_DIR, 'gravity_calibration.yaml'),
            description='Output path for the fitted calibration YAML.',
        ),
        DeclareLaunchArgument(
            'fit_model_type',
            default_value='affine',
            description='Residual model: linear, affine, quadratic, or cubic.',
        ),
        DeclareLaunchArgument(
            'start_delay',
            default_value='5.0',
            description='Countdown (seconds) before the robot starts moving.',
        ),
        DeclareLaunchArgument(
            'check_ground',
            default_value='true',
            description='Reject configurations that touch a virtual ground plane '
                        'at z=ground_height (robot base frame).',
        ),
        DeclareLaunchArgument(
            'ground_height',
            default_value='0.0',
            description='Height (m) of the virtual ground plane in the robot base '
                        'frame. Lower it if the arm may legitimately reach below '
                        'base level.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
