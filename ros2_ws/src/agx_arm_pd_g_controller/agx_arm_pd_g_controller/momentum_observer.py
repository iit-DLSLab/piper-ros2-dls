"""De Luca-style generalized-momentum external torque observer.

Implements the residual filter from De Luca & Mattone: for rigid-body
dynamics M(q)q_ddot + C(q,q_dot)q_dot + g(q) + f(q,q_dot) = tau + tau_ext
(f = passive joint friction/damping), the generalized momentum p = M(q)q_dot
obeys p_dot = tau + tau_ext - g(q) - f(q,q_dot) + C^T(q,q_dot)q_dot (using the
skew-symmetry of M_dot - 2C, i.e. M_dot = C + C^T). Integrating avoids needing
q_ddot:

    r(t) = K_I [ p(t) - p(0) - integral_0^t (tau - beta + r) ds ],
    beta(q,q_dot) = g(q) + f(q,q_dot) - C^T(q,q_dot)q_dot

gives r with dynamics r_dot = K_I(tau_ext - r), i.e. r is a low-pass filtered
estimate of tau_ext with time constant 1/K_I, independent of the control law
driving tau.

C^T(q,q_dot)q_dot = M_dot(q,q_dot)q_dot - C(q,q_dot)q_dot (from M_dot=C+C^T).
MuJoCo exposes C(q,q_dot)q_dot + g(q) as a single bias-force vector
(qfrc_bias, which explicitly excludes joint friction/damping), from which
C(q,q_dot)q_dot is isolated as qfrc_bias(q,q_dot) - qfrc_bias(q,0).
M_dot(q,q_dot)q_dot is the directional derivative of M along the actual
trajectory, so rather than needing the full dM/dq tensor, it's
finite-differenced between consecutive control steps using the mass matrix
already computed for the momentum term - no extra model evaluations. The only
approximation is that finite difference (error O(dt), shrinking with control
rate), not a structural assumption about C.

f(q,q_dot) (viscous damping + Coulomb/dry friction from the model's joint
damping/frictionloss) is qfrc_passive + qfrc_constraint: MuJoCo puts viscous
damping in qfrc_passive but implements dry friction (frictionloss) via the
constraint solver, in qfrc_constraint - both were verified (against mj_inverse
on a trajectory with zero true external torque) to be needed together to
avoid a real, non-shrinking bias in the residual. If the model's damping/
frictionloss don't match the real arm's, that mismatch still leaks into the
residual the same way any other unmodeled dynamics would.
"""

import mujoco
import numpy as np


class MomentumObserver:
    """Per-arm residual filter estimating external joint torque."""

    def __init__(self, model, qpos_indices, qvel_indices, gain):
        if model.nv != len(qvel_indices):
            raise ValueError(
                f"MomentumObserver requires a model containing only the controlled "
                f"joints: got {model.nv} degrees of freedom, {len(qvel_indices)} of "
                "which are controlled. The mass matrix and bias force it reconstructs "
                "the dynamics from are constraint-unaware, so any DOF outside the "
                "controlled joints (e.g. a gripper, especially one linked by an "
                "equality constraint) introduces a real, uncorrected bias into the "
                "residual - it will not average out or shrink with control rate. Use "
                "a model containing only the controlled joints instead."
            )
        self.model = model
        self.data = mujoco.MjData(model)
        self.qpos_indices = qpos_indices
        self.qvel_indices = qvel_indices
        self.gain = np.asarray(gain, dtype=float)
        n = len(qvel_indices)
        self.residual = np.zeros(n)
        self.integral = np.zeros(n)
        self._prev_mass_matrix = None
        self._initialized = False

    def _mass_matrix(self) -> np.ndarray:
        full_mass_matrix = np.zeros((self.model.nv, self.model.nv))
        mujoco.mj_fullM(self.model, self.data, full_mass_matrix)
        return full_mass_matrix[np.ix_(self.qvel_indices, self.qvel_indices)]

    def update(self, qpos, qvel, commanded_torque, dt) -> np.ndarray:
        """Advance the residual filter by one step of size dt; returns tau_ext."""
        qpos = np.asarray(qpos, dtype=float)
        qvel = np.asarray(qvel, dtype=float)
        commanded_torque = np.asarray(commanded_torque, dtype=float)

        self.data.qpos[self.qpos_indices] = qpos
        self.data.qvel[self.qvel_indices] = qvel
        mujoco.mj_forward(self.model, self.data)
        mass_matrix = self._mass_matrix()
        bias = self.data.qfrc_bias[self.qvel_indices].copy()
        friction = (
            self.data.qfrc_passive[self.qvel_indices]
            + self.data.qfrc_constraint[self.qvel_indices]
        ).copy()

        self.data.qvel[self.qvel_indices] = 0.0
        mujoco.mj_forward(self.model, self.data)
        gravity = self.data.qfrc_bias[self.qvel_indices].copy()

        momentum = mass_matrix @ qvel
        coriolis = bias - gravity  # C(q,q_dot)q_dot, exact

        if self._prev_mass_matrix is None:
            mass_matrix_dot_qvel = np.zeros_like(qvel)
        else:
            mass_matrix_dot_qvel = ((mass_matrix - self._prev_mass_matrix) / dt) @ qvel
        self._prev_mass_matrix = mass_matrix

        coriolis_transpose = mass_matrix_dot_qvel - coriolis  # C^T(q,q_dot)q_dot
        beta = gravity - friction - coriolis_transpose

        if not self._initialized:
            self.integral = momentum.copy()
            self.residual = np.zeros_like(momentum)
            self._initialized = True
            return self.residual.copy()

        self.integral = self.integral + (commanded_torque - beta + self.residual) * dt
        self.residual = self.gain * (momentum - self.integral)
        return self.residual.copy()
