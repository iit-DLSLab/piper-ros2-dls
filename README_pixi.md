## Installation with pixi

1. Install [pixi](https://pixi.prefix.dev/latest/installation/) and clone this repository with its submodules (`pyAgxArm` and `agx_arm_ros`):

```bash
git clone --recurse-submodules git@github.com:iit-DLSLab/piper-ros2-dls.git
# or, in an existing clone
git submodule update --init --recursive
```

2. Install the CAN tools used by `can_activate.sh` (system packages, not part of the pixi environment):

```bash
sudo apt install can-utils ethtool
```

3. Activate the environment from anywhere inside the repo (it uses [pixi.toml](./pixi.toml) in the repo root). The first run creates it:

```bash
# ROS2 lyrical (default)
pixi shell   # same as: pixi shell -e lyrical

# ROS2 humble
pixi shell -e humble
```

`pyAgxArm` is installed automatically from the submodule in editable mode, so changes to its code are picked up without reinstalling. Do not `pip install` it by hand.

