import os

from setuptools import setup
from glob import glob

from generate_parameter_library_py.setup_helper import generate_parameter_module

package_name = 'agx_arm_pd_g_controller'
here = os.path.dirname(os.path.realpath(__file__))

generate_parameter_module(
    'agx_arm_pd_g_controller_parameters',
    os.path.join(here, 'agx_arm_pd_g_controller_parameters.yaml'),
)

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models', glob('models/*.xml') + glob('models/*.json')
         + glob('models/*.md')),
        ('share/' + package_name + '/models/assets', glob('models/assets/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Antonio Langella',
    author_email='antonio.langella@iit.it',
    maintainer='Antonio Langella',
    maintainer_email='antonio.langella@iit.it',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Joint-space PD+G controller for the AgileX Piper arm: tracks JointState references via MoveMITMsg with optional MuJoCo-based gravity compensation and a gravity calibration procedure.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pd_g_controller_node = agx_arm_pd_g_controller.pd_g_controller_node:main',
            'gravity_calibration_node = agx_arm_pd_g_controller.gravity_calibration_node:main',
            'fit_calibration = agx_arm_pd_g_controller.fit_calibration:main',
            'friction_hysteresis_check_node = agx_arm_pd_g_controller.friction_hysteresis_check_node:main',
        ],
    },
)
