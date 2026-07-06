"""Fitting and serialization of gravity torque calibrations.

The gravity calibration procedure records joint positions and measured joint
efforts across the workspace (samples .npz with 'qpos' and 'efforts' arrays).
Fitting maps the MuJoCo-predicted gravity torque to the measured effort with a
per-joint polynomial (scipy.optimize.curve_fit, as in
https://github.com/Reimagine-Robotics/piper_control). The result is stored as
numpy-polyval coefficients (degree-descending) in a YAML file, so the
controller can evaluate it without scipy.
"""

import datetime
from typing import Dict, Sequence

import numpy as np
import yaml

from agx_arm_pd_g_controller.gravity_compensation import GravityCompensationModel

# Polynomial degree per model type; coefficients are stored degree-descending
# as accepted by numpy.polyval.
MODEL_TYPES = {
    "linear": 1,
    "affine": 1,
    "quadratic": 2,
    "cubic": 3,
}

_FIT_BOUND = 100.0


def fit_calibration(
    samples_path: str,
    model: GravityCompensationModel,
    model_type: str = "affine",
    model_path: str = "",
) -> Dict:
    """Fit per-joint polynomials mapping MuJoCo gravity torque to measured effort."""
    from scipy import optimize

    if model_type not in MODEL_TYPES:
        raise ValueError(
            f"Unknown model type '{model_type}'. Expected one of {list(MODEL_TYPES)}."
        )
    degree = MODEL_TYPES[model_type]

    npz = np.load(samples_path)
    if "qpos" not in npz or "efforts" not in npz:
        raise ValueError(
            f"Samples file must contain 'qpos' and 'efforts' arrays. "
            f"Existing keys: {list(npz.keys())}"
        )
    qpos = npz["qpos"]
    efforts = npz["efforts"]

    sim_tau = np.array([model.raw_gravity_torque(q) for q in qpos])

    def poly(x, *coeffs):
        return np.polyval(coeffs, x)

    joints: Dict[str, Dict] = {}
    for joint_idx, joint_name in enumerate(model.joint_names):
        n_params = degree + 1
        bounds = ([-_FIT_BOUND] * n_params, [_FIT_BOUND] * n_params)
        if model_type == "linear":
            # Pure gain, no offset: pin the constant term to (almost) zero.
            bounds[0][-1] = -1e-12
            bounds[1][-1] = 1e-12

        opt_params, _, infodict, mesg, ier = optimize.curve_fit(
            poly,
            sim_tau[:, joint_idx],
            efforts[:, joint_idx],
            p0=np.zeros(n_params),
            bounds=bounds,
            full_output=True,
        )
        residual = float(np.abs(infodict["fvec"]).sum())
        joints[joint_name] = {
            "coeffs": [float(c) for c in opt_params],
            "residual_abs_sum": residual,
        }
        print(
            f"{joint_name}: {model_type} coeffs={np.round(opt_params, 4).tolist()} "
            f"residual_abs_sum={residual:.4f} ({mesg.strip()}, ier={ier})"
        )

    return {
        "model_type": model_type,
        "fitted_on": datetime.datetime.now().isoformat(timespec="seconds"),
        "model_path": model_path,
        "num_samples": int(qpos.shape[0]),
        "joints": joints,
    }


def save_calibration(calibration: Dict, path: str) -> None:
    with open(path, "w") as f:
        yaml.safe_dump(calibration, f, sort_keys=False)


def load_calibration(path: str) -> Dict:
    with open(path, "r") as f:
        calibration = yaml.safe_load(f)
    if not isinstance(calibration, dict) or "joints" not in calibration:
        raise ValueError(f"'{path}' is not a valid gravity calibration file.")
    return calibration
