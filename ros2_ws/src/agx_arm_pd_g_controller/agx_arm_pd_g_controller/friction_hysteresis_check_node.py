"""Quick friction/effort-estimation check for the AgileX Piper arm.

Commands the arm to a fixed, stiff MIT-mode configuration. For each joint,
runs: measure -> perturb+ -> return -> measure -> perturb- -> return ->
measure. All three measurements are taken at the exact same configuration;
the perturb-and-return in between only exercises that joint (in each
direction) to expose direction-dependent stiction/friction hysteresis.
Reports, per joint, the range |max(mean effort) - min(mean effort)| in N*m
across the three measurements -> a quick proxy for how much friction
pollutes torque/wrench estimation at that joint's configuration.

Run (after sourcing the workspace):
    python3 friction_check_node.py
"""

import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from agx_arm_msgs.msg import MoveMITMsg

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
BASE_CONFIG = np.array([0.0, 0.4, -0.7, 0.0, 0.23, 0.22])

# Stiff MIT gains for a static hold (stiffer than the teleoperation
# defaults of [5, 5, 5, 5.6, 20, 6] / [0.8]*6).
KP = [20.0, 20.0, 20.0, 20.0, 30.0, 20.0]
KD = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

PERTURBATION = 0.1  # rad
PERTURB_DWELL = 0.3  # s, dwell at the perturbed config before returning
MOVE_DURATION = 1.5  # s, time to interpolate between configurations
SETTLE_WAIT = 2.0  # s, wait after reaching a configuration before recording
RECORD_DURATION = 2.0  # s, how long to record effort for the measurement
CONTROL_FREQUENCY = 100.0  # Hz


class FrictionHysteresisCheckNode(Node):

    def __init__(self):
        super().__init__("friction_check")

        self.output_topic = "control/move_mit"
        self.feedback_topic = "feedback/joint_states"

        self._lock = threading.Lock()
        self._efforts = {}
        self._positions = {}

        self.publisher = self.create_publisher(MoveMITMsg, self.output_topic, 1)
        self.subscription = self.create_subscription(
            JointState, self.feedback_topic, self._feedback_callback, 10
        )

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _feedback_callback(self, msg: JointState) -> None:
        has_effort = len(msg.effort) == len(msg.name)
        has_position = len(msg.position) == len(msg.name)
        with self._lock:
            for i, name in enumerate(msg.name):
                if has_effort:
                    self._efforts[name] = msg.effort[i]
                if has_position:
                    self._positions[name] = msg.position[i]

    def _get_efforts(self):
        with self._lock:
            try:
                return np.array([self._efforts[j] for j in JOINT_NAMES])
            except KeyError:
                return None

    def _get_positions(self):
        with self._lock:
            try:
                return np.array([self._positions[j] for j in JOINT_NAMES])
            except KeyError:
                return None

    def _command(self, positions) -> None:
        msg = MoveMITMsg()
        for i in range(len(JOINT_NAMES)):
            msg.joint_index.append(i + 1)
            msg.p_des.append(float(positions[i]))
            msg.v_des.append(0.0)
            msg.kp.append(KP[i])
            msg.kd.append(KD[i])
            msg.torque.append(0.0)
        self.publisher.publish(msg)

    def _move_to(self, start, target) -> None:
        dt = 1.0 / CONTROL_FREQUENCY
        num_steps = max(1, int(MOVE_DURATION * CONTROL_FREQUENCY))
        for step in range(num_steps):
            alpha = (step + 1) / num_steps
            self._command(start + alpha * (target - start))
            time.sleep(dt)

    def _hold_and_measure(self, target) -> np.ndarray:
        """Hold `target`, wait SETTLE_WAIT, then record mean effort over RECORD_DURATION."""
        dt = 1.0 / CONTROL_FREQUENCY
        deadline = time.time() + SETTLE_WAIT
        while time.time() < deadline:
            self._command(target)
            time.sleep(dt)

        samples = []
        deadline = time.time() + RECORD_DURATION
        while time.time() < deadline:
            self._command(target)
            efforts = self._get_efforts()
            if efforts is not None:
                samples.append(efforts)
            time.sleep(dt)

        return np.mean(samples, axis=0)

    def _run(self) -> None:
        try:
            self._check()
        except Exception as e:  # noqa: BLE001 - report failure before shutdown
            self.get_logger().error(f"Friction check failed: {e}")
        finally:
            if rclpy.ok():
                rclpy.try_shutdown()

    def _check(self) -> None:
        self.get_logger().info(f"Waiting for feedback on '{self.feedback_topic}'...")
        while self._get_efforts() is None or self._get_positions() is None:
            time.sleep(0.1)

        current = self._get_positions()
        self.get_logger().info(
            f"Moving from {np.round(current, 3).tolist()} to base config "
            f"{BASE_CONFIG.tolist()}..."
        )
        self._move_to(current, BASE_CONFIG)

        ranges = np.zeros(len(JOINT_NAMES))
        for j in range(len(JOINT_NAMES)):
            name = JOINT_NAMES[j]

            # All three measurements are taken at the exact same BASE_CONFIG;
            # the perturb-and-return in between only exercises the joint (in
            # each direction) to expose direction-dependent stiction/friction
            # hysteresis at that fixed configuration.
            measure1 = self._hold_and_measure(BASE_CONFIG)

            plus_target = BASE_CONFIG.copy()
            plus_target[j] += PERTURBATION
            self.get_logger().info(f"[{name}] perturbing + ({plus_target[j]:.3f} rad)...")
            self._move_to(BASE_CONFIG, plus_target)
            time.sleep(PERTURB_DWELL)
            self._move_to(plus_target, BASE_CONFIG)

            measure2 = self._hold_and_measure(BASE_CONFIG)

            minus_target = BASE_CONFIG.copy()
            minus_target[j] -= PERTURBATION
            self.get_logger().info(f"[{name}] perturbing - ({minus_target[j]:.3f} rad)...")
            self._move_to(BASE_CONFIG, minus_target)
            time.sleep(PERTURB_DWELL)
            self._move_to(minus_target, BASE_CONFIG)

            measure3 = self._hold_and_measure(BASE_CONFIG)

            three = np.array([measure1[j], measure2[j], measure3[j]])
            ranges[j] = np.max(three) - np.min(three)
            self.get_logger().info(
                f"[{name}] efforts at base config: m1={measure1[j]:.4f}, "
                f"m2={measure2[j]:.4f}, m3={measure3[j]:.4f}  -> "
                f"range={ranges[j]:.4f} N*m"
            )

        self.get_logger().info("=== Friction check summary (N*m) ===")
        for j, name in enumerate(JOINT_NAMES):
            self.get_logger().info(f"  {name}: range = {ranges[j]:.4f}")


def main(args=None):
    rclpy.init(args=args)
    node = FrictionHysteresisCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
