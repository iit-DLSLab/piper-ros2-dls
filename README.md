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
- **agx_arm_pd_g_controller**: A PD+G controller for the arm, based on the official AgileX ROS 2 HAL
- **dls2_interface**: Standard messages definition for DLS2
- **dls2_piper_bridge**: DLS2 hardware-abstraction-layer using the `pyAgxArm` API

## Installation

You can follow a readme based on conda [here](./README_conda.md) or rosdep [here](./README_rosdep.md) 

## ROS2-DLS HAL (with custom messages)

To control the arm using ROS2 **with custom messages from DLS**, a single python script is provided to launch the HAL

```bash
python3 launch_piper_hal.py
```

These custom messages can be found [here](./ros2_ws/src/dls2_interface/msg/) and are automatically compiled when launching the script above.

**Important**: We believe that some internal scaling are happening for Kp/Kd values for joints1-2-3.
We identified these values, and we apply an inverse scaling in the hal (see [the hal code](./ros2_ws/src/dls2_piper_bridge/dls2_piper_bridge/piper_hal.py)).
In this way, gains in simulation matches more closely the one on the real robot.
Note that this scaling is **not applied in the official ROS2 HAL**, so it must be applied manually when using the official ROS2 HAL.

## ROS2-Official HAL + MoveIt (with standard ros2 messages)

To control the arm using ROS2  **with standard ros2 messages**, the official ROS 2 HAL can be employed. 

Before using the arm, CAN-bus communication must be manually enabled

```bash
bash ./src/agx_arm_ros/scripts/can_activate.sh
```

A convenient launch file is provided in `agx_arm_ctrl` to run the arm control node together with the MoveIt framework

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py arm_type:=piper_l effector_type:=agx_gripper follow:=true
```

> [!WARNING]
> Running this command will move the arm to the zero configuration.
> Ensure that there are no obstacles nearby and always start from a nearby configuration.


## Maintainer

This repository is maintained by [Giulio Turrisi](https://github.com/giulioturrisi) and [Antonio Langella](https://github.com/AntoSave).
