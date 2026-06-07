#!/usr/bin/env python3
"""Interactive joint zero calibration helper for Piper arms."""

import argparse
import sys
import time
from pathlib import Path
from platform import system

try:
    from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
    from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config


PIPER_MODELS = {
    "piper": ArmModel.PIPER,
    "piper_h": ArmModel.PIPER_H,
    "piper_l": ArmModel.PIPER_L,
    "piper_x": ArmModel.PIPER_X,
}


def default_interface():
    platform_system = system()
    if platform_system == "Windows":
        return "agx_cando", "0"
    if platform_system == "Darwin":
        return "slcan", "/dev/ttyACM0"
    return "socketcan", "can0"


def parse_args():
    interface, channel = default_interface()
    parser = argparse.ArgumentParser(
        description=(
            "Set the current Piper joint position as zero by calling "
            "robot.calibrate_joint()."
        )
    )
    parser.add_argument(
        "--model",
        choices=sorted(PIPER_MODELS),
        default="piper",
        help="Piper arm model to calibrate.",
    )
    parser.add_argument(
        "--firmware",
        choices=[PiperFW.DEFAULT, PiperFW.V183, PiperFW.V188],
        default=PiperFW.DEFAULT,
        help="Main controller firmware family.",
    )
    parser.add_argument(
        "--interface",
        default=interface,
        help="CAN backend, for example socketcan, agx_cando, or slcan.",
    )
    parser.add_argument(
        "--channel",
        default=channel,
        help="CAN channel, for example can0, 0, or /dev/ttyACM0.",
    )
    parser.add_argument(
        "--joint",
        type=int,
        default=1,
        help="Joint to calibrate: 1-6 for one joint, or 255 for all joints.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=1.0,
        help="Seconds to wait for the controller calibration response.",
    )
    parser.add_argument(
        "--no-disable",
        action="store_true",
        help="Do not disable the selected joint before prompting for manual positioning.",
    )
    parser.add_argument(
        "--reenable",
        action="store_true",
        help="Re-enable the selected joint after successful calibration.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip confirmation prompts. Use only when the arm is already positioned.",
    )
    return parser.parse_args()


def confirm(prompt, assume_yes=False):
    if assume_yes:
        return True
    answer = input(prompt).strip().lower()
    return answer in {"y", "yes"}


def make_config(args):
    return create_agx_arm_config(
        robot=PIPER_MODELS[args.model],
        firmeware_version=args.firmware,
        interface=args.interface,
        channel=args.channel,
    )


def print_current_angles(robot):
    angles = robot.get_joint_angles()
    if angles is None:
        print("Current joint angles: unavailable")
        return
    print("Current joint angles:", angles.msg)


def main():
    args = parse_args()
    if args.joint not in [1, 2, 3, 4, 5, 6, 255]:
        print("--joint must be one of 1, 2, 3, 4, 5, 6, or 255", file=sys.stderr)
        return 2

    all_joints = args.joint == 255
    if all_joints:
        print(
            "WARNING: --joint 255 calibrates all joints with the controller's "
            "offset procedure. Use only with the required pre-calibration pose."
        )
        if not confirm("Continue with all-joint calibration? [y/N] ", args.yes):
            print("Cancelled.")
            return 1

    cfg = make_config(args)
    robot = AgxArmFactory.create_arm(cfg)

    try:
        print(
            "Connecting to {model} on {interface}:{channel} "
            "with firmware {firmware}...".format(
                model=args.model,
                interface=args.interface,
                channel=args.channel,
                firmware=args.firmware,
            )
        )
        robot.connect()
        print_current_angles(robot)

        if not args.no_disable:
            print("Disabling joint {}...".format(args.joint))
            robot.disable(args.joint)
            time.sleep(0.2)

        if not args.yes:
            if all_joints:
                input(
                    "Move the arm to the documented all-joint calibration pose, "
                    "then press Enter..."
                )
            else:
                input(
                    "Manually move joint {} to its desired zero position, "
                    "then press Enter...".format(args.joint)
                )

        print("Sending calibration request...")
        ok = robot.calibrate_joint(args.joint, timeout=args.timeout)
        if not ok:
            print("Calibration failed or timed out.", file=sys.stderr)
            return 1

        print("Calibration succeeded.")
        if args.reenable:
            print("Re-enabling joint {}...".format(args.joint))
            robot.enable(args.joint)
        return 0
    finally:
        robot.disconnect()


if __name__ == "__main__":
    sys.exit(main())
