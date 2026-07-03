## Installation with rosdep

0. Install ROS 2 Jazzy and initialize `rosdep`.

1. Clone this repository including submodules

    ```bash
    git clone --recurse-submodules git@github.com:iit-DLSLab/piper-ros2-dls.git
    cd piper-ros2-dls
    ```

2. Source ROS 2 and install rosdep dependencies

    ```bash
    source /opt/ros/jazzy/setup.bash
    rosdep install -iyr --from-paths ros2_ws/src
    ```

3. Install any system dependencies required by the official AgileX packages

    ```bash
    sudo apt install can-utils ethtool
    ```

4. Build and source the ROS 2 workspace

    ```bash
    cd ros2_ws
    colcon build
    source install/setup.bash
    ```
