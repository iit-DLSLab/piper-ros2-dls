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

### `firmware_gain_scaling`

The firmware applies its own multiplier to the MIT gains. Measured on this arm
by regressing `feedback/joint_states.effort` on the command's own terms:

```
effort(t) = a·kp·(p_des(t) − q(t)) + b·kd·(v_des(t) − q̇(t)) + c·τ_ff(t) + d
```

with `c` pinned to 1 and `a`, `b` read off as the multiplier, over the two
bags with strong excitation (R² 0.85–0.99 with the correction below). This
regression touches no dynamics model — every term is a recorded command or a
direct measurement — so it is unaffected by any error in the identified
model, and it can be (and is) done independently of, and in either order
relative to, identifying armature/inertia/friction (see
`models/README.md` §4 for why the two are orthogonal by construction).

The regression pairs each command with the nearest-in-time feedback sample,
which assumes zero command-to-effort delay. That assumption is false by
~12.5 ms — sweeping an explicit shift finds R² maximized there consistently
across every joint and both bags, matching the ~15 ms lag found independently
on the dynamics side — and the zero-lag fit under-reports the arm's true
instantaneous multiplier by up to 50% on `kd`.

That true multiplier is deliberately **not** what is shipped, though. This
parameter feeds `_commanded_torques`, which pairs the current measured state
with the most recently *published* command — exactly the zero-lag assumption
the uncorrected regression makes. Plugging the delay-corrected multiplier into
that same zero-lag pairing overcorrects: measured on `multijoint_bag_2` (held
out, free-space), the momentum-observer residual gets *worse* with the
"more correct" gain (0.519 → 0.627 Nm pooled), and recovers to 0.543 once the
setpoint is paired with a matching 12.5 ms delay — confirming the gap is a
magnitude/timing mismatch, not a wrong number. `firmware_gain_scaling` is set
to the zero-lag fit, the one self-consistent with how the code actually pairs
its inputs:

| joint | `kp` (shipped, zero-lag) | `kd` (shipped, zero-lag) |
|---|---|---|
| joint1 | 22.6 ± 0.6 | 15.6 ± 0.1 |
| joint2 | 22.0 ± 1.7 | 18.5 ± 0.0 |
| joint3 | 18.8 ± 0.4 | 17.2 ± 0.3 |
| joint4 | 1.2 ± 0.2 | 0.6 ± 0.1 |
| joint5 | 1.4 ± 0.1 | 0.9 ± 0.0 |
| joint6 | 1.4 ± 0.4 | 0.3 ± 0.0 |

See `models/README.md` §4 for the delay-corrected values and the full
held-out comparison. These are set in `config/pd_g_controller.yaml`; the
parameter default stays at `1.0` because the values are firmware- and
arm-specific. They only affect the commanded-torque reconstruction, so they
are inert while `torque_source` is `measured`.

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
