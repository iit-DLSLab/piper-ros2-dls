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

## Installation

1. install [miniforge](https://github.com/conda-forge/miniforge/releases) (x86_64 or arm64 depending on your platform)

2. create an environment using the file in the folder [installation](https://github.com/iit-DLSLab/piper-ros2-dls/tree/main/installation):

    `conda env create -f installation/mamba_environment.yml`


3. clone the other submodules:

    `git submodule update --init --recursive`

4. activate the env and install the submodule

```bash
conda activate piper_ros2_env
pip install -e pyAgxArm/
```

