"""Gravity torque calibration procedure for the AgileX Piper arm.

!!! THIS NODE MOVES THE ROBOT through its full range of motion. Make sure the
area around the arm is clear of obstacles and people before launching. !!!

The node commands the arm (via MoveMITMsg on the driver's control/move_mit
topic) through a Halton sequence of collision-free joint configurations while
recording measured joint positions and efforts from feedback/joint_states.
It then:
  1. saves the raw samples to a .npz file (qpos, efforts, target_qpos),
  2. fits a per-joint polynomial mapping the MuJoCo-predicted gravity torque
     to the measured effort,
  3. writes the fitted calibration YAML read by the PD+G controller
     (gravity_compensation.calibration_path parameter).

Ported to ROS 2 from https://github.com/Reimagine-Robotics/piper_control
(generate_samples.py).
"""

import math
import os
import threading
import time

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from agx_arm_msgs.msg import MoveMITMsg

from agx_arm_pd_g_controller.calibration import fit_calibration, save_calibration
from agx_arm_pd_g_controller.gravity_compensation import (
    DEFAULT_JOINT_NAMES,
    GravityCompensationModel,
)
from agx_arm_pd_g_controller.mujoco_model import (
    HaltonSampler,
    get_colliding_body_pairs,
    load_mujoco_model,
)

# Conservative MIT gains for the calibration moves (from piper_control).
DEFAULT_KP = [5.0, 5.0, 5.0, 5.6, 20.0, 6.0]
DEFAULT_KD = [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]
DEFAULT_OUTPUT_DIR = os.path.expanduser("~/.ros/agx_arm_pd_g_controller")

_MAX_CONSECUTIVE_REJECTIONS = 500


class GravityCalibrationNode(Node):
    """Collects gravity calibration samples and fits the calibration model."""

    def __init__(self):
        super().__init__("gravity_calibration")

        self.joints = list(
            self.declare_parameter("joints", list(DEFAULT_JOINT_NAMES)).value
        )
        self.model_path = self.declare_parameter("model_path", "").value
        if not self.model_path:
            raise RuntimeError(
                "Parameter 'model_path' is required (MuJoCo .xml or .urdf/.xacro model)."
            )
        self.output_topic = self.declare_parameter("output_topic", "control/move_mit").value
        self.feedback_topic = self.declare_parameter("feedback_topic", "feedback/joint_states").value
        self.num_samples = self.declare_parameter("num_samples", 50).value
        self.control_frequency = self.declare_parameter("control_frequency", 200.0).value
        self.move_duration = self.declare_parameter("move_duration", 2.5).value
        self.kp = list(self.declare_parameter("kp", DEFAULT_KP).value)
        self.kd = list(self.declare_parameter("kd", DEFAULT_KD).value)
        self.joint_limit_margin = self.declare_parameter("joint_limit_margin", 0.05).value
        self.samples_output = self.declare_parameter(
            "samples_output", os.path.join(DEFAULT_OUTPUT_DIR, "gravity_samples.npz")
        ).value
        self.calibration_output = self.declare_parameter(
            "calibration_output", os.path.join(DEFAULT_OUTPUT_DIR, "gravity_calibration.yaml")
        ).value
        self.fit_model_type = self.declare_parameter("fit_model_type", "affine").value
        self.start_delay = self.declare_parameter("start_delay", 5.0).value
        self.return_to_start = self.declare_parameter("return_to_start", True).value
        self.check_ground = self.declare_parameter("check_ground", True).value
        self.ground_height = self.declare_parameter("ground_height", 0.0).value
        self.wall_x_pos = self.declare_parameter("wall_x_pos", math.nan).value
        self.wall_x_neg = self.declare_parameter("wall_x_neg", math.nan).value
        self.wall_y_pos = self.declare_parameter("wall_y_pos", math.nan).value
        self.wall_y_neg = self.declare_parameter("wall_y_neg", math.nan).value
        self.settle_velocity_threshold = self.declare_parameter(
            "settle_velocity_threshold", 0.02
        ).value
        self.settle_timeout = self.declare_parameter("settle_timeout", 3.0).value
        self.settle_check_period = self.declare_parameter("settle_check_period", 0.05).value

        if len(self.kp) != len(self.joints) or len(self.kd) != len(self.joints):
            raise RuntimeError("'kp' and 'kd' must have one entry per joint.")

        self.get_logger().info(f"Loading MuJoCo model from '{self.model_path}'...")
        self.gravity_model = GravityCompensationModel(
            model_path=self.model_path,
            joint_names=self.joints,
        )
        # Separate model for collision checking, with a ground plane at
        # z=ground_height and optional walls so samples below base level
        # (table mounts) or beyond the walls are rejected too.
        walls = {
            "x_pos": None if math.isnan(self.wall_x_pos) else self.wall_x_pos,
            "x_neg": None if math.isnan(self.wall_x_neg) else self.wall_x_neg,
            "y_pos": None if math.isnan(self.wall_y_pos) else self.wall_y_pos,
            "y_neg": None if math.isnan(self.wall_y_neg) else self.wall_y_neg,
        }
        self.mj_model = load_mujoco_model(
            self.model_path,
            add_ground_plane=self.check_ground,
            ground_height=self.ground_height,
            walls=walls,
        )
        self.mj_data = mujoco.MjData(self.mj_model)
        collision_joint_ids = [self.mj_model.joint(n).id for n in self.joints]
        self.collision_qpos_indices = self.mj_model.jnt_qposadr[collision_joint_ids]
        # Joints not in `self.joints` (e.g. a gripper) are not sampled during
        # calibration, but still need *some* qpos for collision checking.
        # Defaulting them to 0 is wrong in general: 0 can sit at the edge of
        # a joint's range (e.g. a gripper fully closed), which can register a
        # permanent, arm-configuration-independent self-collision between its
        # own links and reject every single sample. The range midpoint is a
        # much safer default for this.
        self._collision_qpos_default = np.zeros(self.mj_model.nq)
        self._collision_qpos_default[self.mj_model.jnt_qposadr] = self.mj_model.jnt_range.mean(
            axis=1
        )

        self._feedback_lock = threading.Lock()
        self._positions = {}
        self._velocities = {}
        self._efforts = {}

        self.publisher = self.create_publisher(MoveMITMsg, self.output_topic, 1)
        self.subscription = self.create_subscription(
            JointState, self.feedback_topic, self._feedback_callback, 1
        )

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._stop_event = threading.Event()
        self._worker.start()

    def _feedback_callback(self, msg: JointState) -> None:
        has_effort = len(msg.effort) == len(msg.name)
        has_velocity = len(msg.velocity) == len(msg.name)
        with self._feedback_lock:
            for i, name in enumerate(msg.name):
                self._positions[name] = msg.position[i]
                if has_effort:
                    self._efforts[name] = msg.effort[i]
                if has_velocity:
                    self._velocities[name] = msg.velocity[i]

    def _get_state(self):
        """Measured (qpos, efforts, velocity) ordered per self.joints, or None if incomplete."""
        with self._feedback_lock:
            try:
                qpos = np.array([self._positions[j] for j in self.joints])
                efforts = np.array([self._efforts[j] for j in self.joints])
                velocity = np.array([self._velocities[j] for j in self.joints])
            except KeyError:
                return None
        return qpos, efforts, velocity

    def _command(self, positions) -> None:
        msg = MoveMITMsg()
        for i, _ in enumerate(self.joints):
            msg.joint_index.append(i + 1)
            msg.p_des.append(float(positions[i]))
            msg.v_des.append(0.0)
            msg.kp.append(self.kp[i])
            msg.kd.append(self.kd[i])
            msg.torque.append(0.0)
        self.publisher.publish(msg)

    def _move_to(self, start, target) -> bool:
        """Linearly interpolate from start to target."""
        dt = 1.0 / self.control_frequency
        num_steps = max(1, int(self.move_duration * self.control_frequency))
        for step in range(num_steps):
            if self._stop_event.is_set():
                return False
            alpha = (step + 1) / num_steps
            interp = start + alpha * (target - start)
            self._command(interp)
            time.sleep(dt)
        return True

    def _wait_until_settled(self, target):
        """Hold `target` until measured velocity settles, then return (qpos, efforts).

        Returns None if stopped externally. Recording gravity-torque samples
        while the arm is still moving contaminates them with inertial,
        centrifugal/Coriolis, and friction effects unrelated to gravity, so we
        wait for the joints to (near-)stop before taking the measurement.
        """
        deadline = time.time() + self.settle_timeout
        while time.time() < deadline:
            if self._stop_event.is_set():
                return None
            self._command(target)
            state = self._get_state()
            if state is not None:
                qpos, efforts, velocity = state
                if np.all(np.abs(velocity) < self.settle_velocity_threshold):
                    return qpos, efforts
            time.sleep(self.settle_check_period)
        self.get_logger().warning(
            f"Timed out after {self.settle_timeout}s waiting for the arm to settle; "
            "recording the sample anyway."
        )
        state = self._get_state()
        return (state[0], state[1]) if state is not None else None

    def _is_collision_free(self, qpos) -> bool:
        self.mj_data.qpos[:] = self._collision_qpos_default
        self.mj_data.qpos[self.collision_qpos_indices] = qpos
        contacts = get_colliding_body_pairs(self.mj_model, self.mj_data)
        if contacts:
            self.get_logger().debug(f"Rejected sample, contacts: {dict(contacts)}")
        return not contacts

    def _run(self) -> None:
        try:
            self._calibrate()
        except Exception as e:  # noqa: BLE001 - report any failure before shutdown
            self.get_logger().error(f"Calibration failed: {e}")
        finally:
            if rclpy.ok():
                self.get_logger().info("Shutting down.")
                rclpy.try_shutdown()

    def _calibrate(self) -> None:
        self.get_logger().info(f"Waiting for feedback on '{self.feedback_topic}'...")
        while self._get_state() is None:
            if self._stop_event.is_set():
                return
            time.sleep(0.1)

        start_state = self._get_state()
        initial_qpos = start_state[0]

        self.mj_data.qpos[:] = self._collision_qpos_default
        self.mj_data.qpos[self.collision_qpos_indices] = initial_qpos
        contacts = get_colliding_body_pairs(self.mj_model, self.mj_data)
        if contacts:
            self.get_logger().warning(
                f"Model reports contacts at the current configuration: {dict(contacts)}. "
                "Collision filtering may be over-conservative; sampling will still "
                "reject any configuration with contacts."
            )

        self.get_logger().warning(
            "=== GRAVITY CALIBRATION: THE ROBOT IS ABOUT TO MOVE through its full "
            "range of motion. Clear the area around the arm NOW! ==="
        )
        for remaining in range(int(round(self.start_delay)), 0, -1):
            self.get_logger().warning(f"Starting in {remaining} s...")
            time.sleep(1.0)

        margin = self.joint_limit_margin
        limits_min = self.gravity_model.joint_range[:, 0] + margin
        limits_max = self.gravity_model.joint_range[:, 1] - margin
        halton = HaltonSampler(limits_min, limits_max)

        samples_qpos, samples_efforts, samples_target = [], [], []

        sample_count = 0
        rejections = 0
        while sample_count < self.num_samples and not self._stop_event.is_set():
            target = halton.sample()
            if not self._is_collision_free(target):
                rejections += 1
                if rejections >= _MAX_CONSECUTIVE_REJECTIONS:
                    raise RuntimeError(
                        f"{rejections} consecutive samples rejected as colliding; "
                        "check the model's collision geometry."
                    )
                continue
            rejections = 0
            sample_count += 1
            self.get_logger().info(
                f"Sample {sample_count}/{self.num_samples}: moving to "
                f"{np.round(target, 2).tolist()}"
            )

            current = self._get_state()[0]
            if not self._move_to(current, target):
                return

            settled = self._wait_until_settled(target)
            if settled is None:
                return
            qpos, efforts = settled
            samples_qpos.append(qpos)
            samples_efforts.append(efforts)
            samples_target.append(target.copy())

        if self.return_to_start:
            self.get_logger().info("Returning to the initial configuration...")
            current = self._get_state()[0]
            self._move_to(current, initial_qpos)

        if not samples_qpos:
            raise RuntimeError("No samples recorded.")

        os.makedirs(os.path.dirname(self.samples_output) or ".", exist_ok=True)
        np.savez(
            self.samples_output,
            qpos=np.array(samples_qpos),
            efforts=np.array(samples_efforts),
            target_qpos=np.array(samples_target),
        )
        self.get_logger().info(
            f"Saved {len(samples_qpos)} samples to '{self.samples_output}'"
        )

        self.get_logger().info(f"Fitting '{self.fit_model_type}' calibration model...")
        calibration = fit_calibration(
            self.samples_output,
            self.gravity_model,
            model_type=self.fit_model_type,
            model_path=self.model_path,
        )
        os.makedirs(os.path.dirname(self.calibration_output) or ".", exist_ok=True)
        save_calibration(calibration, self.calibration_output)
        self.get_logger().info(
            f"Calibration written to '{self.calibration_output}'. Pass it to the "
            "controller via the gravity_compensation.calibration_path parameter "
            "(calibration_file launch argument)."
        )

    def stop(self) -> None:
        self._stop_event.set()


def main(args=None):
    rclpy.init(args=args)
    node = GravityCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
