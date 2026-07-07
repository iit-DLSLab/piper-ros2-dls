# agx_arm_pd_g_controller

Joint-space **PD+G controller** for the AgileX Piper arm. A node subscribes to a
`sensor_msgs/JointState` reference and republishes it as
`agx_arm_msgs/MoveMITMsg` (MIT mode) with per-joint `kp`/`kd` gains and a
feed-forward torque that can include **MuJoCo-based gravity compensation**
evaluated at the *measured* joint configuration (from the driver's
`feedback/joint_states`).

The robot model is taken from `agx_arm_description` (URDF/xacro converted to
MuJoCo on the fly: visual meshes are stripped, collision STLs kept). A plain
MuJoCo `.xml` model is also accepted anywhere a model file is expected.

The package also provides a **gravity calibration procedure** (ported from
[Reimagine-Robotics/piper_control](https://github.com/Reimagine-Robotics/piper_control))
that estimates, on the real arm, the per-joint mapping between the model-predicted
gravity torque and the effort actually needed by the motors.

## Dependencies

All dependencies are declared in `package.xml` and resolvable with rosdep:

```bash
rosdep install --from-paths src --ignore-src -y
```

## Running the controller

```bash
# PD only (same behavior as the old agx_arm_compliance_bridge):
ros2 launch agx_arm_pd_g_controller pd_g_controller.launch.py

# PD + gravity compensation (defaults: robot_model:=piper_l, no gripper):
ros2 launch agx_arm_pd_g_controller pd_g_controller.launch.py \
    enable_gravity_compensation:=true

# With gripper model, and a calibration file from the procedure below:
ros2 launch agx_arm_pd_g_controller pd_g_controller.launch.py \
    enable_gravity_compensation:=true robot_model:=piper_l use_gripper:=true \
    calibration_file:=$HOME/.ros/agx_arm_pd_g_controller/gravity_calibration.yaml
```

Launch arguments: `params_file`, `namespace`, `robot_model` (any model in
`agx_arm_description`: `piper`, `piper_l`, `piper_h`, `piper_x`, `nero`, ...;
default `piper_l`), `use_gripper`, `model_file` (explicit
`.urdf`/`.xacro`/`.xml`, overrides `robot_model`/`use_gripper`),
`enable_gravity_compensation`, `calibration_file`.

Topics (relative, remap/namespace as needed):

| Topic | Type | Direction | Description |
|---|---|---|---|
| `control/move_mit_joint_states` | `sensor_msgs/JointState` | in | joint reference (position, optional velocity) |
| `feedback/joint_states` | `sensor_msgs/JointState` | in | measured state used for gravity compensation |
| `control/move_mit` | `agx_arm_msgs/MoveMITMsg` | out | MIT command to the driver |

When gravity compensation is enabled the node does **not** publish commands
until feedback has been received for every controlled joint.

## Gravity calibration procedure

> **WARNING: the robot moves through its full range of motion.** Clear the
> area around the arm before launching. There is a `start_delay` countdown
> (default 5 s) after the first feedback message before motion starts.

With the arm driver running (and the PD+G controller **not** publishing):

```bash
ros2 launch agx_arm_pd_g_controller gravity_calibration.launch.py \
    num_samples:=50 robot_model:=piper_l use_gripper:=false
```

The node visits `num_samples` collision-free configurations (Halton sequence,
checked against the model's collision geometry). Collision checking also
includes a virtual ground plane at `ground_height:=0.0` in the base frame, so
table-mounted arms never sample below base level — lower it or set
`check_ground:=false` if the arm may reach below its base. Optional invisible
walls can also be added at `wall_x_pos`/`wall_x_neg`/`wall_y_pos`/`wall_y_neg`
(each a coordinate in meters, base frame; empty disables that side) to keep
the arm away from nearby obstacles such as a wall, monitor, or another robot —
the four sides are independent, so the box need not be symmetric.

After moving to each target, the node holds position and waits for the
measured joint velocities to settle below `settle_velocity_threshold`
(default 0.02 rad/s, up to `settle_timeout` seconds) before recording that
sample's position and effort. Recording while still moving would contaminate
the sample with inertial/friction effects unrelated to gravity.

For each recorded sample it computes the MuJoCo-predicted gravity torque
`tau_sim = qfrc_bias(qpos)` (zero velocity, so this is exactly the gravity
term) and fits, per joint, a polynomial mapping it to the effort `tau_meas`
actually measured on the motor:

```
tau_meas ≈ f(tau_sim)   with e.g. f(x) = a·x + b   (the default "affine" model)
```

`scipy.optimize.curve_fit` finds `a`, `b` (or the higher-order coefficients
for `quadratic`/`cubic`) minimizing the residual over all samples for that
joint; `a` mostly captures fixed effects such as the firmware's 4× MIT torque
scaling on joints 1–3 (see below), while `b` captures a constant per-joint
bias (friction, encoder offset, etc.). The result is written to:

- `~/.ros/agx_arm_pd_g_controller/gravity_samples.npz` — raw samples
  (`qpos`, `efforts`, `target_qpos`);
- `~/.ros/agx_arm_pd_g_controller/gravity_calibration.yaml` — fitted per-joint
  coefficients (`fit_model_type`: `linear`, `affine` (default), `quadratic`,
  `cubic`). This is the file the controller reads via `calibration_file`; at
  runtime it evaluates `torque = f(tau_sim)` with `numpy.polyval`, no scipy
  required.

To refit offline from existing samples (e.g. with a different model type):

```bash
ros2 run agx_arm_pd_g_controller fit_calibration \
    --samples ~/.ros/agx_arm_pd_g_controller/gravity_samples.npz \
    --model $(ros2 pkg prefix --share agx_arm_description)/agx_arm_urdf/piper_l/urdf/piper_l_description.urdf \
    --model-type quadratic \
    -o ~/.ros/agx_arm_pd_g_controller/gravity_calibration.yaml
```

## Firmware torque scaling (important)

Piper firmware **≤ 1.8.post2 amplifies MIT torque commands on joints 1–3 by
4×** ([piper_sdk Q&A](https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/Q%26A.MD)).
When **no calibration file** is used, the raw MuJoCo torque is multiplied by
the `gravity_compensation.torque_scaling` parameter, whose default
`[0.25, 0.25, 0.25, 1.0, 1.0, 1.0]` is the safe choice for old/unknown
firmware. If your firmware is newer than 1.8.post2, set it to all `1.0` in the
params YAML. You can check the firmware version with the piper SDK
(`GetPiperFirmwareVersion`).

With a calibration file the fitted coefficients absorb this scaling
automatically — but a calibration is only valid for the firmware (and payload)
it was recorded with.
