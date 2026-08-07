"""MuJoCo-based gravity torque prediction for the AgileX Piper arm.

The raw gravity torque is MuJoCo's ``qfrc_bias`` evaluated at the measured
joint configuration (zero velocity, so it reduces to the gravity term). The
raw torque is then scaled per joint (defaults to 1.0). Note: Piper firmware
<= 1.8.post2 amplifies MIT torque commands on joints 1-3 by 4x; the
controller's parameter YAML sets 0.25 there. See the package README.

Ported from https://github.com/Reimagine-Robotics/piper_control.
"""

import os
from typing import Callable, Optional, Sequence

import mujoco
import numpy as np

from agx_arm_pd_g_controller.mujoco_model import load_mujoco_model

DEFAULT_JOINT_NAMES = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
)

class GravityCompensationModel:
    """Predicts per-joint gravity compensation torques."""

    def __init__(
        self,
        model_path: str,
        joint_names: Sequence[str] = DEFAULT_JOINT_NAMES,
        torque_scaling: Optional[Sequence[float]] = None,
        package_resolver: Optional[Callable[[str], str]] = None,
        weld_joints_except: Optional[Sequence[str]] = None,
    ):
        self.model = load_mujoco_model(
            model_path, package_resolver, weld_joints_except=weld_joints_except
        )
        self.data = mujoco.MjData(self.model)
        self.joint_names = tuple(joint_names)

        model_joints = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(self.model.njnt)
        ]
        missing = [name for name in self.joint_names if name not in model_joints]
        if missing:
            raise ValueError(
                f"Joints {missing} not found in MuJoCo model '{model_path}'. "
                f"Available joints: {model_joints}"
            )

        joint_indices = [self.model.joint(name).id for name in self.joint_names]
        self.qpos_indices = self.model.jnt_qposadr[joint_indices]
        self.qvel_indices = self.model.jnt_dofadr[joint_indices]
        self.joint_range = self.model.jnt_range[joint_indices].copy()

        if torque_scaling is None:
            torque_scaling = [1.0] * len(self.joint_names)
        if len(torque_scaling) != len(self.joint_names):
            raise ValueError(
                f"torque_scaling has {len(torque_scaling)} entries for "
                f"{len(self.joint_names)} joints."
            )
        self._torque_scaling = np.asarray(torque_scaling, dtype=float)


    def raw_gravity_torque(self, qpos: Sequence[float]) -> np.ndarray:
        """MuJoCo gravity torque (qfrc_bias at zero velocity) for the arm joints."""
        self.data.qpos[self.qpos_indices] = qpos
        mujoco.mj_forward(self.model, self.data)
        return self.data.qfrc_bias[self.qvel_indices].copy()

    def predict(self, qpos: Sequence[float]) -> np.ndarray:
        """Command torques compensating gravity at the given configuration."""
        return self.raw_gravity_torque(qpos) * self._torque_scaling
