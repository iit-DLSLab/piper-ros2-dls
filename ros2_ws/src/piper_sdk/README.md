# piper_sdk

This package is a ROS 2 wrapper for the official `pyAgxArm` Python SDK.

During `colcon build`, this wrapper installs the `pyAgxArm` Python import package from the top-level `pyAgxArm` git submodule into the ROS 2 install space.
This avoids using `pip install --break-system-packages` on ROS 2 Jazzy systems.
