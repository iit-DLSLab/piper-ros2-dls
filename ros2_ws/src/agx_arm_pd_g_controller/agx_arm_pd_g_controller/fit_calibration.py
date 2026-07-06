"""CLI to (re)fit a gravity calibration YAML from an existing samples .npz.

Example:
    ros2 run agx_arm_pd_g_controller fit_calibration \
        --samples ~/.ros/agx_arm_pd_g_controller/gravity_samples.npz \
        --model /path/to/piper_description.urdf \
        --output ~/.ros/agx_arm_pd_g_controller/gravity_calibration.yaml \
        --model-type affine
"""

import argparse
import os

from agx_arm_pd_g_controller.calibration import (
    MODEL_TYPES,
    fit_calibration,
    save_calibration,
)
from agx_arm_pd_g_controller.gravity_compensation import (
    DEFAULT_JOINT_NAMES,
    GravityCompensationModel,
)


def main():
    parser = argparse.ArgumentParser(
        description="Fit a gravity calibration YAML from recorded samples."
    )
    parser.add_argument("--samples", required=True, help="Input samples .npz file")
    parser.add_argument(
        "--model", required=True, help="Robot model (.xml MJCF, .urdf, or .xacro)"
    )
    parser.add_argument("-o", "--output", required=True, help="Output calibration YAML")
    parser.add_argument(
        "--model-type",
        default="affine",
        choices=sorted(MODEL_TYPES),
        help="Residual model type (default: affine)",
    )
    parser.add_argument(
        "--joint-names",
        nargs="+",
        default=list(DEFAULT_JOINT_NAMES),
        help="Joint names, in the order recorded in the samples file",
    )
    args = parser.parse_args()

    model = GravityCompensationModel(
        model_path=args.model,
        joint_names=args.joint_names,
    )
    calibration = fit_calibration(
        args.samples, model, model_type=args.model_type, model_path=args.model
    )
    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    save_calibration(calibration, args.output)
    print(f"Calibration written to {args.output}")


if __name__ == "__main__":
    main()
