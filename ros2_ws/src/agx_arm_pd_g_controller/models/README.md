# piper_l dynamics models

Identified MuJoCo models of the piper_l arm, used by `pd_g_controller_node` for
gravity compensation and for the momentum-observer external-torque estimate.

| file | contents |
|---|---|
| `piper_l_identified_no_spacer.xml` | **default.** Arm + gripper, no plastic spacer. |
| `piper_l_identified_with_spacer.xml` | Same model with the 17.0 g plastic spacer fitted between the joint-6 flange and the gripper. |
| `piper_l_friction.json` | Full friction model (asymmetric, load-dependent). Reference only — its direction-averaged part is already baked into both XMLs. |
| `assets/` | STL meshes, referenced relatively so the models load from anywhere. |

Both models are 8-DOF (6 arm joints + 2 gripper fingers under an equality
constraint) and carry no fitted parameter that differs between them: the
no-spacer variant is derived from the with-spacer one by exact rigid-body
subtraction, not by a separate fit.

## Source data

Four bags recorded on the physical arm at 200 Hz, all free-space (no contact),
all with the plastic spacer fitted:

| bag | duration | role in the fit |
|---|---|---|
| `move_and_hold_bag_2` | 255 s | Gravity. 66 sustained zero-velocity hold segments spanning joint1 −0.31…1.87, joint2 0.46…2.47, joint3 −2.52…−0.07 rad. |
| `multiple_frequencies_bag_2` | 120 s | Multi-frequency excitation for inertia and armature. |
| `constant_vel_2` | 77 s | Constant-velocity sweeps (q̈ ≈ 0) isolating friction from inertia. |
| `multijoint_bag_2` | 41 s | **Held out.** Cross-joint coupling, highest accelerations; never entered the fit. |

A fifth bag from an earlier session was used as a second held-out set to check
that the result survives a change of recording session.

## Procedure

### 1. Model corrections (no fitting)

Starting from the vendor model, six defects were corrected against CAD and
against direct measurement. Each is a known-wrong fact, not a fitted quantity:

- **link2/link3 orientation** taken from the CAD URDF (−7.7800° / −102.7800°)
  rather than the vendor model's whole-degree rounding, which acted as a
  constant −2.22° offset on joint2 and joint3.
- **link4/link5 frames** set by direct visual alignment of the meshes. The CAD
  URDF's joint4 origin disagrees with the meshes it ships: measured as mean
  surface gap at the link3/link4 mating face, the URDF value gives 6.27 mm
  against 1.82 mm for the values used here.
- **RealSense camera removed.** The vendor model lumps a camera and stand that
  are not on the arm into link6's inertial; this was worth 1.17 Nm RMS on
  joint2 and 0.89 Nm on joint3.
- **Joint travel** widened to the real range. The vendor limits are narrower
  than the arm's actual motion (joint4 ±1.832 vs ±2.21 recorded), and MuJoCo
  returns joint-limit constraint forces outside them.
- **link1 mass** set to the CAD value of 0.71 kg (vendor model: 0.11 kg).
- **Measured masses**: gripper base + both fingers 542.5 g, plastic spacer
  17.0 g, both weighed. Inertia tensors for links 2–5 are kept from the vendor
  model, which matches mesh-uniform values; the URDF's own tensors for those
  links are inflated 4–5× and were not used.

### 2. Parameter identification

The measured `effort` field of `feedback/joint_states` is the fit target. The
alternative — reconstructing torque from the MIT command as
`kp*(p_des−q)+kd*(v_des−q̇)+τ_ff` — understates joints 1–3 by roughly 25×,
because the firmware applies its own multiplier to kp/kd. `effort` needs no
assumption about that.

`q`, `q̇` and `q̈` come from a Savitzky-Golay differentiator applied to
**position**, not from the reported velocity: the latter is quantised at
1e-3 rad/s, and central-differencing it yields an acceleration that is almost
entirely noise (4.9–5.9 rad/s² RMS on joints 4–6 where the true value is ~0).
The torque is smoothed with the same window to stay in phase, and a 15 ms lag
in reported effort is removed.

Inverse dynamics is exactly linear in each body's standard inertial parameters
(mass, m·c, and — with the principal frame held fixed — the inertia diagonal),
in joint armature, and in the friction coefficients. The fit is therefore a
single weighted-ridge linear least-squares solve with no local minima, rather
than a search. It is regularised toward the CAD/measured prior with a
per-parameter physical scale, and the regularisation weight is selected on a
block-wise validation split restricted to solutions that stay physically
admissible — masses near CAD, COM shifts under 15 mm, valid inertia tensors.

Friction is fitted as asymmetric viscous + Coulomb plus a load-dependent term
(`|τ|·sign(q̇)`, the standard geared-joint effect, which matters most on
joint2). Its direction-averaged part is written into each joint's
`damping`/`frictionloss`; the full model is in `piper_l_friction.json`.

A mounting misalignment of 0.70° was identified and is expressed as a rotation
of `link0`, so `option/gravity` stays standard.

### 3. No-spacer variant

Derived from the identified model by exact subtraction, since every bag was
recorded with the spacer fitted and refitting without it would push the
spacer's mass into other parameters. link6's fitted lumped inertial is split
back into the arm-side flange (known), the spacer (weighed, known geometry) and
the gripper assembly (the remainder, carrying the fit's correction); the gripper
assembly is moved proximal by the 12.3 mm spacer thickness; link6 is recomposed
from flange + moved gripper, and the gripper base geom and both fingers are
shifted by the same amount. Total mass drops by exactly 17.0 g and the fingers
return to their pre-spacer position.

### 4. Firmware gain multipliers

Identified from the same bags, as a by-product rather than as part of the
model: the firmware does not apply the MIT `kp`/`kd` it is sent as-is.

**Strategy.** Per joint, per sample, regress

```
effort(t) = a·kp·(p_des(t) − q(t)) + b·kd·(v_des(t) − q̇(t)) + c·τ_ff(t) + d
```

against the recorded `kp`, `kd`, `p_des`, `v_des`, `τ_ff` (from `control/move_mit`)
and measured `q`, `q̇`, `effort` (from `feedback/joint_states`) — a linear
least-squares fit of the control law's own coefficients directly against its
measured output, with no dynamics model anywhere in it. `c` is pinned to 1
(the feedforward term is already in true Nm and passes through at unity; this
also removes a regressor that is collinear with the position-error term on the
low-excitation bags — see below) and `a`, `b` are read off as the multiplier.
Only the two bags with strong excitation
(`multiple_frequencies_bag_2`, `multijoint_bag_2`) are used, for the same
collinearity reason.

**Effect of command-to-effort delay.** The regression above pairs each
`move_mit` sample with the feedback sample nearest in time. That implicitly
assumes zero delay between commanding a joint and its effort being reported,
which is false: sweeping an explicit time shift of `effort` relative to the
command finds R² maximized at a consistent **+12.5 ms** across every joint and
both bags (e.g. joint1 in `multijoint_bag_2`: R² 0.925 at 0 ms → 0.957 at
+12.5 ms), matching, independently, the ~15 ms lag `identify_linear.py` found
and corrected for on the dynamics side. Left uncorrected, this biases the
fitted multiplier low — by up to 50% on `kd` — because a delay between a
control-law error signal and its measured effect always attenuates a
zero-lag correlation between them. The table below is fit at the located
delay.

**Effect of velocity quantization.** `q̇` from `feedback/joint_states` is
quantized at 1e-3 rad/s (the same defect that made raw-differenced
acceleration unusable for the dynamics fit, §2). Using it instead of a
Savitzky-Golay derivative of position changes `b` by at most 14% — real, but
an order of magnitude smaller than the delay effect, and within the
between-bag spread already in the table. Not corrected for, since the recorded
`v_des − q̇` term is exactly what the firmware itself would have used.

**Effect of dynamic model correctness: none.** The regression above never
evaluates `M(q)`, gravity, armature or friction — every term in it is either a
recorded command or a direct measurement. It is therefore unaffected by any
error in the identified model, including all the defects in §1. This is a
deliberate property of using measured `effort` as the target rather than, say,
a MuJoCo-predicted torque: had the fit instead compared a *reconstructed*
commanded torque against a *model-predicted* one, every kinematic and
inertial defect in §1 would have leaked into the gain estimate, which is
exactly the failure mode that produced the ~25× error the original
(uncorrected) approach in this project's early history was built on.

**Should armature/friction be identified first?** No — the two identification
tasks are independent by construction, not just in principle: §2's dynamics
fit and this gain fit both use measured `effort` as their target, but they
regress it against disjoint predictor sets (rigid-body terms from `mj_inverse`
for one, the control law's own command terms for the other), so neither result
enters the other and there is no ordering constraint. This is why §2 needed no
`firmware_gain_scaling` value at all to identify armature, inertia and
friction. The one place a model *does* feed into a fit is internal to §2 itself:
its load-dependent friction term uses the prior model's own predicted torque as
a proxy for joint load, so that specific term (not armature/inertia/mass/COM)
is only as good as the kinematic and inertial corrections already applied
before it runs.

| joint | `kp` multiplier | `kd` multiplier |
|---|---|---|
| joint1 | 27.7 ± 0.2 | 25.6 ± 0.4 |
| joint2 | 24.1 ± 0.5 | 23.6 ± 0.7 |
| joint3 | 26.6 ± 0.7 | 24.3 ± 0.1 |
| joint4 | 1.6 ± 0.0 | 1.2 ± 0.1 |
| joint5 | 1.6 ± 0.0 | 1.4 ± 0.1 |
| joint6 | 1.7 ± 0.0 | 1.1 ± 0.1 |

These are what `firmware_gain_scaling` is set to in the controller config.
They are not used in fitting the models — `effort` is the fit target precisely
so that no gain assumption enters. Joint2's value (and, to a lesser extent,
joint1/joint3's) is only identifiable on bags with strong excitation: on the
hold bags the feedforward term carries most of the torque and correlates with
the pose error at ρ = 0.6–0.7, the fit goes collinear, and the estimate ranges
from −1 to +24 with R² collapsing to 0.01.

## Results

Torque-prediction RMS, Nm pooled over joints, against the previously used model:

| bag | previous model | identified |
|---|---|---|
| `move_and_hold_bag_2` | 0.509 | 0.371 |
| `multiple_frequencies_bag_2` | 0.834 | 0.418 |
| `constant_vel_2` | 0.460 | 0.299 |
| `multijoint_bag_2` (held out) | 1.506 | 0.574 |
| earlier session (held out) | 0.544 | 0.304 |

Momentum-observer residual on free-space data, where any non-zero value is
error: 0.386 → 0.267, 1.135 → 0.801, and 2.163 → 1.371 Nm on the three bags,
with the same observer settings and torque input.

## Notes and limitations

- **Joint limits are disabled** in both models. The momentum observer reads its
  friction term from `qfrc_passive + qfrc_constraint`, and MuJoCo puts
  joint-limit forces in `qfrc_constraint` too, so a model with limits feeds
  spurious torques into the estimate whenever a joint reaches one. Re-enable
  them for simulation use, not for the controller.
- **Armature is a lower bound.** It rises monotonically as the differentiation
  window widens, the signature of errors-in-variables attenuation from residual
  acceleration noise. A dedicated high-acceleration excitation would pin it
  down.
- **The rigid-body model does not represent the arm's ~15–19 Hz structural
  resonance.** Above it the observer's residual degenerates into a phantom
  torque proportional to velocity; see `external_torque_estimation.residual_filter_hz`
  in the controller parameters.
- Joint2 keeps the largest residual (0.71 Nm RMS held-out against a 7.4 Nm
  signal). Joint1 carries no gravity torque at all yet still shows ~0.31 Nm, so
  a large part of what remains on every joint is stiction, which a sliding
  friction model cannot represent.
