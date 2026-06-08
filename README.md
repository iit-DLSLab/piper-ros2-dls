## Overview

ROS 2 workspace for controlling AgileX Piper arms from ROS 2 or DLS controllers.

The repository is based on AgileX's new driver stack:

- `pyAgxArm` for the Python CAN driver
- `agx_arm_ros` for the official ROS 2 control, message, description, and MoveIt packages

The local ROS package named `piper_sdk` is kept as a thin wrapper around `pyAgxArm`, so the SDK is installed by `colcon build` without using `pip install --break-system-packages`.

## Repository Contents

This workspace contains the following packages:

- **piper_sdk**: ROS 2 wrapper around the `pyAgxArm` git submodule, automatically installed by `colcon build`
- **agx_arm_ros**: Official AgileX ROS 2 packages for control, messages, description, and MoveIt
- **dls2_interface**: Standard messages definition for DLS2
- **dls2_piper_bridge**: DLS2 hardware-abstraction-layer using the `pyAgxArm` API

## Prerequisites

- Install ROS 2 Jazzy and initialize `rosdep`.

- Clone this repository including submodules

    ```bash
    git clone --recurse-submodules git@github.com:iit-DLSLab/piper-ros2-dls.git
    cd piper-ros2-dls
    ```

- Source ROS 2 and install rosdep dependencies

    ```bash
    source /opt/ros/jazzy/setup.bash
    rosdep install -y --ignore-src --from-paths ros2_ws/src
    ```

- Install any system dependencies required by the official AgileX packages

    ```bash
    sudo apt install can-utils ethtool
    ```

- Build and source the ROS 2 workspace

    ```bash
    cd ros2_ws
    colcon build
    source install/setup.bash
    ```

## Usage

Before using the arm, CAN-bus communication must be manually enabled.
Assuming the current folder is `ros2_ws`, this can be done with

```bash
bash ./src/agx_arm_ros/scripts/can_activate.sh
```

### DLS 2 bridge

The DLS 2 bridge is independent of the official `agx_arm_ros` packages.
To run it after building and sourcing the workspace use

```bash
ros2 run dls2_piper_bridge piper_hal
```

The DLS2 bridge accepts the CAN interface as a ROS parameter:

```bash
ros2 run dls2_piper_bridge piper_hal --ros-args -p can_port:=can0
```

### ROS 2 MoveIt

A convenient launch file is provided in `agx_arm_ctrl` to run the arm control node together with the MoveIt framework

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py arm_type:=piper_l effector_type:=agx_gripper follow:=true
```

> [!WARNING]
> Running this command will move the arm to the zero configuration.
> Ensure that there are no obstacles nearby and always start from a nearby configuration.

## Maintainer

This repository is maintained by [Giulio Turrisi](https://github.com/giulioturrisi) and [Antonio Langella](https://github.com/AntoSave).
