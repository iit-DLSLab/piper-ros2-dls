# agx_arm_pd_g_controller

Joint-space **PD+G controller** for the AgileX Piper arm. A node subscribes to a
`sensor_msgs/JointState` reference and republishes it as
`agx_arm_msgs/MoveMITMsg` (MIT mode) with per-joint `kp`/`kd` gains and a
feed-forward torque that can include **MuJoCo-based gravity compensation**
evaluated at the *measured* joint configuration (from the driver's
`feedback/joint_states`).

The package ships **identified dynamics models** of the piper_l arm in
[`models/`](models/), and uses them by default. They replace the earlier
per-joint gravity calibration procedure: rather than fitting a scalar mapping
between predicted and measured gravity torque, the model's own kinematics,
masses, inertias, armature and friction were identified against recorded
motion. See [`models/README.md`](models/README.md) for the procedure and its
limitations.

Any model in `agx_arm_description` can still be used instead (URDF/xacro
converted to MuJoCo on the fly: visual meshes stripped, collision STLs kept),
as can any plain MuJoCo `.xml`.

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

# With the spacer-equipped model instead of the default:
ros2 launch agx_arm_pd_g_controller pd_g_controller.launch.py \
    enable_gravity_compensation:=true \
    model_file:=$(ros2 pkg prefix --share agx_arm_pd_g_controller)/models/piper_l_identified_with_spacer.xml
```

Launch arguments: `params_file`, `namespace`, `robot_model` (any model in
`agx_arm_description`: `piper`, `piper_l`, `piper_h`, `piper_x`, `nero`, ...;
default `piper_l`), `use_gripper`, `model_file` (explicit
`.urdf`/`.xacro`/`.xml`, overrides `robot_model`/`use_gripper`),
`enable_gravity_compensation`.

Topics (relative, remap/namespace as needed):

| Topic | Type | Direction | Description |
|---|---|---|---|
| `control/move_mit_joint_states` | `sensor_msgs/JointState` | in | joint reference (position, optional velocity) |
| `feedback/joint_states` | `sensor_msgs/JointState` | in | measured state used for gravity compensation |
| `control/move_mit` | `agx_arm_msgs/MoveMITMsg` | out | MIT command to the driver |

When gravity compensation is enabled the node does **not** publish commands
until feedback has been received for every controlled joint.

## Robot model

With no `model_file:=`, the launch file prefers this package's
`models/<robot_model>_identified_no_spacer.xml` when one exists, and otherwise
falls back to `agx_arm_description`. For `piper_l` that means the identified
model is used automatically. Two variants ship:

| model | contents |
|---|---|
| `piper_l_identified_no_spacer.xml` | **default** — arm + gripper |
| `piper_l_identified_with_spacer.xml` | same, plus the 17.0 g plastic spacer between the joint-6 flange and the gripper |

Joint limits are deliberately disabled in both. The momentum observer reads its
friction term from `qfrc_passive + qfrc_constraint`, and MuJoCo puts
joint-limit forces in `qfrc_constraint` too, so a limited model injects
spurious torque into the estimate whenever a joint reaches a limit. Re-enable
them for simulation, not for the controller.

## Firmware torque scaling (important)

Piper firmware **≤ 1.8.post2 amplifies MIT torque commands on joints 1–3 by
4×** ([piper_sdk Q&A](https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/Q%26A.MD)).
The MuJoCo gravity torque is multiplied by the
`gravity_compensation.torque_scaling` parameter, whose default
`[0.25, 0.25, 0.25, 1.0, 1.0, 1.0]` is the safe choice for old/unknown
firmware. If your firmware is newer than 1.8.post2, set it to all `1.0` in the
params YAML. You can check the firmware version with the piper SDK
(`GetPiperFirmwareVersion`).

## External torque estimation

`external_torque_estimation` publishes a per-joint estimate of the torque
applied to the arm from outside on `feedback/external_torque`. With
`method: momentum_observer` it runs a De Luca generalized-momentum residual
filter over the identified model; `commanded_torque_diff` is the cheaper
alternative that cannot separate the arm's own inertial torque from contact.

### `torque_source`

Selects the actuator torque the observer is driven with:

- `commanded` — reconstructs `kp*(p_des-q) + kd*(v_des-q_dot) + t_ff` from the
  last MIT command, scaled by `firmware_gain_scaling`.
- `measured` — uses `feedback/joint_states`' effort field directly, needing no
  assumption about the firmware's gain handling.

The observer's evidence for contact is a comparison between measured
generalized momentum and the momentum implied by integrating the applied
torque — the torque is a model *input*, not a reference being differenced, so
it should be as accurate as available. An understated torque does not make the
estimate conservative, it makes it blind: in a closed-loop simulation with a
known external torque, `commanded` at `firmware_gain_scaling: 1.0` misses a
2.5 Nm push on joint2 entirely, because the firmware's proportional term ramps
up by exactly the amount the reconstruction fails to count.

### `firmware_command_delay` and `firmware_gain_scaling`

The firmware neither applies the MIT gains it is sent as-is, nor acts on a
command the instant it is published. Both were measured on this arm by
regressing `feedback/joint_states.effort` on the command's own terms:

```
effort(t) = a·kp·(p_des(t−Δ) − q(t)) + b·kd·(v_des(t−Δ) − q̇(t)) + c·τ_ff(t−Δ) + d
```

with `c` pinned to 1, over the two bags with strong excitation. This regression
touches no dynamics model — every term is a recorded command or a direct
measurement — so it is unaffected by any error in the identified model, and it
is independent of identifying armature/inertia/friction in either order (see
`models/README.md` §4).

Sweeping Δ finds R² maximized at **11.5 ms**, consistently across five joints
and both bags. The delay is a real property of the arm, so it is **modelled**
(`firmware_command_delay`) rather than absorbed: `_commanded_torques`
reconstructs against the setpoint the firmware is actually acting on, which is
what lets the gains below be the arm's true multipliers rather than values
distorted to compensate for unmodelled timing.

| joint | `kp` multiplier | `kd` multiplier |
|---|---|---|
| joint1 | 27.3 ± 0.1 | 24.6 ± 0.2 |
| joint2 | 23.9 ± 0.6 | 23.2 ± 0.6 |
| joint3 | 25.8 ± 0.6 | 23.7 ± 0.0 |
| joint4 | 1.6 ± 0.0 | 1.2 ± 0.1 |
| joint5 | 1.6 ± 0.0 | 1.4 ± 0.1 |
| joint6 | 1.7 ± 0.1 | 1.0 ± 0.1 |

Both are set in `config/pd_g_controller.yaml`; the parameter defaults stay at
`1.0` / `0.0` because the values are firmware- and arm-specific. They affect
only the commanded-torque reconstruction — never the torque actually sent to
the arm — and are inert while `torque_source` is `measured`.
See [this readme](models/README.md) for more details about model identification.`

This supersedes an earlier conclusion in this README that no such scaling
applied, and that `dls2_piper_bridge`'s `PIPER_SCALE_KP`/`PIPER_SCALE_KD` were
purely an interface convention. The scaling is real and roughly that size on
joints 1–3. Note that the estimate for joint2 (and, to a lesser extent,
joint1/joint3) is only trustworthy on bags with strong excitation: where the
feedforward term carries most of the torque and correlates with the pose
error, the fit is collinear and returns anything between −1 and +24.

### `residual_filter_hz`

A second-order Butterworth low-pass on the published estimate (`0` disables).

The rigid-body model cannot represent the arm's ~15–19 Hz structural
resonance. Above it the observer's cancellation breaks down and the residual
degenerates into a phantom torque proportional to velocity, `K_I*M(q)*q_dot`,
measured at 6.5 N·m·s/rad on joint2. Force-reflected to a leader arm that is
negative damping, and it destabilised closed-loop teleoperation on joint2 in
both `torque_source` modes.

The artefact and the signal are a decade apart — the observer's own bandwidth
is `momentum_observer_gain` (10 rad/s ≈ 1.6 Hz), so it cannot carry contact
information near the resonance anyway. At 5 Hz the filter attenuates 15.5 Hz
by 10× and 18.8 Hz by 15× for 45 ms of added step delay, leaving 1 Hz at
0.999. Prefer it to lowering `momentum_observer_gain`, which trades
attenuation against contact latency 1:1.
