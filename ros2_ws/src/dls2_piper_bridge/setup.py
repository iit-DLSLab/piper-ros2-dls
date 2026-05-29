from setuptools import setup
import os

package_name = 'dls2_piper_bridge'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    author='Giulio Turrisi',
    author_email='giulio.turrisi@example.com',
    maintainer='Giulio Turrisi',
    maintainer_email='giulio.turrisi@example.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Piper HAL bridge for DLS2 (ROS2 node).',
    license='Apache License, Version 2.0',
    entry_points={'console_scripts': ['piper_hal = dls2_piper_bridge.piper_hal:main']},
)
