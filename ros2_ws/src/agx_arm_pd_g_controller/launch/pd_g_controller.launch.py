import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_model_path(context) -> str:
    """Model precedence: explicit model_file arg > identified MJCF > description.

    An identified model shipped in this package's models/ directory is
    preferred over agx_arm_description's stock model, because it carries the
    corrected kinematics, measured masses and fitted friction that gravity
    compensation and the momentum observer both depend on (see
    models/README.md). The no-spacer variant is the default; pass model_file:=
    explicitly to select the with-spacer one or bypass both.

    agx_arm_description lays models out as
    agx_arm_urdf/<robot_model>/urdf/<robot_model>_description.urdf and
    <robot_model>_with_gripper_description.xacro for every robot (piper,
    piper_l, piper_h, piper_x, nero, ...).
    """
    model_file = context.launch_configurations['model_file']
    if model_file:
        return model_file

    robot_model = context.launch_configurations['robot_model']
    models = os.path.join(
        get_package_share_directory('agx_arm_pd_g_controller'), 'models')
    for candidate in (f'{robot_model}_identified_no_spacer.xml',
                      f'{robot_model}_identified.xml'):
        path = os.path.join(models, candidate)
        if os.path.exists(path):
            return path

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
        executable='pd_g_controller_node',
        name='agx_arm_pd_g_controller',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'gravity_compensation.enable':
                    context.launch_configurations['enable_gravity_compensation'].lower()
                    in ('true', '1'),
                'gravity_compensation.model_path': resolve_model_path(context),
                'gravity_compensation.calibration_path':
                    context.launch_configurations['calibration_file'],
            },
        ],
    )
    return [node]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('agx_arm_pd_g_controller'),
        'config',
        'pd_g_controller.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_config,
            description='Path to the YAML file with the controller parameters.',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace to push the node (and its topics) under.',
        ),
        DeclareLaunchArgument(
            'robot_model',
            default_value='piper_l',
            description='Robot model from agx_arm_description (piper, piper_l, '
                        'piper_h, piper_x, nero, ...) used for gravity compensation.',
        ),
        DeclareLaunchArgument(
            'use_gripper',
            default_value='false',
            description='Use the robot model variant with gripper.',
        ),
        DeclareLaunchArgument(
            'model_file',
            default_value='',
            description='Explicit robot model file (.xml MJCF, .urdf, or .xacro) for '
                        'gravity compensation. Overrides use_gripper.',
        ),
        DeclareLaunchArgument(
            'enable_gravity_compensation',
            default_value='false',
            description='Enable MuJoCo-based gravity compensation.',
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='',
            description='Gravity calibration YAML produced by gravity_calibration.launch.py. '
                        'Empty: use the torque_scaling parameter instead.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
