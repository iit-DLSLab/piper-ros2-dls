# dls2_piper_bridge

A ROS 2 package that provides a lightweight bridge between Piper hardware SDK to DLS2 topics.

## Run the bridge

After building the package and sourcing `install/setup.bash` you can run the node with:

```bash
ros2 run dls2_piper_bridge piper_hal
```

## Robot state visualization demo

A demo for visualizing the robot state via `plotjuggler`.
The demo can be run with:

```bash
ros2 launch dls2_piper_bridge joint_state_demo.launch.py
```
