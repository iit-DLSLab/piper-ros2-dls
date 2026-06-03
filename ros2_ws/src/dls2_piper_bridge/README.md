# dls2_piper_bridge

A ROS 2 package that provides a lightweight bridge between the `pyAgxArm`
hardware SDK and DLS2 topics.

This package keeps the existing DLS2 topic interface while the repository moves
to the new AgileX driver stack. The official ROS 2 packages under
`agx_arm_ros` can also be launched directly for the standard AgileX ROS control,
message, description, and MoveIt interfaces.

## Run the bridge

After building the package and sourcing `install/setup.bash` you can run the node with:

```bash
ros2 run dls2_piper_bridge piper_hal
```

The CAN interface defaults to `can0` and can be overridden with:

```bash
ros2 run dls2_piper_bridge piper_hal --ros-args -p can_port:=can0
```

## Robot state visualization demo

A demo for visualizing the robot state via `plotjuggler`.
The demo can be run with:

```bash
ros2 launch dls2_piper_bridge joint_state_demo.launch.py
```
