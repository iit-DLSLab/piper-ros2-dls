## Installation with rosdep

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
