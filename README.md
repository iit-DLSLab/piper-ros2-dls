## Overview

A wrapper around the `piper_sdk` repository that provides a script for communicating with DLS controllers.

## Repository Contents

This workspace contains the following packages:

- **piper_sdk**: Wrapper around the `piper_sdk` git submodule, automatically installed by `colcon build`
- **dls2_interface**: Standard messages definition for DLS2
- **dls2_piper_bridge**: ROS 2 hardware-abstraction-layer for DLS2 controllers

## Prerequisites

- Install ROS 2 Jazzy and initialize `rosdep`.

- Clone this repository including submodules:

```bash
git clone --recurse-submodules git@github.com:iit-DLSLab/piper-ros2-dls.git
cd piper-ros2-dls
```

- Install rosdep dependencies:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install -y --ignore-src --from-paths ros2_ws/src
```

## Build

Build the ROS 2 workspace with:

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

## Usage

Activate CAN-bus communication for the arm:

```bash
./piper_sdk/piper_sdk/can_activate.sh
```

Run the DLS2 hardware-abstraction-layer launcher script:

```bash
source ros2_ws/install/setup.bash
ros2 run dls2_piper_bridge piper_hal
```

## Maintainer

This repository is maintained by [Giulio Turrisi](https://github.com/giulioturrisi).
