import os

from setuptools import setup
from glob import glob

from generate_parameter_library_py.setup_helper import generate_parameter_module

package_name = 'agx_arm_compliance_bridge'
here = os.path.dirname(os.path.realpath(__file__))

generate_parameter_module(
    'agx_arm_compliance_bridge_parameters',
    os.path.join(here, 'agx_arm_compliance_bridge_parameters.yaml'),
)

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
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
    description='Republishes JointState commands as MoveMITMsg with configurable compliance gains.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'compliance_bridge_node = agx_arm_compliance_bridge.compliance_bridge_node:main',
        ],
    },
)
