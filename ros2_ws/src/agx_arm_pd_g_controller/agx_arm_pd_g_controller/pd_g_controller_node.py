from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from agx_arm_msgs.msg import MoveMITMsg

from agx_arm_pd_g_controller.agx_arm_pd_g_controller_parameters import pd_g_controller


class PdGControllerNode(Node):
    """Joint-space PD(+G) controller.

    Republishes JointState references as MoveMITMsg with per-joint PD gains,
    optionally adding a MuJoCo-based gravity compensation torque evaluated at
    the measured joint configuration.
    """

    def __init__(self):
        super().__init__("agx_arm_pd_g_controller")

        self.param_listener = pd_g_controller.ParamListener(self)
        self.params = self.param_listener.get_params()

        if not self.params.joints:
            raise RuntimeError(
                "Parameter 'joints' is empty. Set it in the node's YAML config to the "
                "ordered list of joint names to control."
            )
        # The Piper SDK's MIT joint_index is 1-based (joint1 == 1, ..., joint6 == 6).
        self.joint_indices = {name: i for i, name in enumerate(self.params.joints, start=1)}
        # 0-based position in params.joints, matching the ordering of the
        # gravity_compensation/remote_force_feedforward torque_scaling arrays.
        self.joint_list_index = {name: i for i, name in enumerate(self.params.joints)}

        self.gravity_model = None
        if self.params.gravity_compensation.enable and not self.params.gravity_compensation.model_path:
            raise RuntimeError(
                "gravity_compensation.enable is true but gravity_compensation.model_path "
                "is empty. Set it to a MuJoCo .xml or a .urdf/.xacro model file."
            )
        if self.params.gravity_compensation.model_path:
            # Load whenever a model is configured (not only when enabled), so
            # gravity compensation can be toggled at runtime.
            from agx_arm_pd_g_controller.gravity_compensation import GravityCompensationModel

            self.gravity_model = GravityCompensationModel(
                model_path=self.params.gravity_compensation.model_path,
                joint_names=self.params.joints,
                torque_scaling=self.params.gravity_compensation.torque_scaling,
            )
            self.get_logger().info(
                f"Gravity model loaded from '{self.params.gravity_compensation.model_path}' "
                f"with torque scaling {list(self.params.gravity_compensation.torque_scaling)}"
            )

        self.measured_positions = {}
        self.measured_velocities = {}
        self.measured_efforts = {}
        self.latest_setpoint = None
        # Last MIT command actually sent per joint (p_des, v_des, kp, kd, torque),
        # used to reconstruct what the firmware's own kp*(p_des-p)+kd*(v_des-v)+t_ff
        # law is contributing to measured_efforts, so external torque estimation can
        # subtract it out instead of misreading it as contact force.
        self.last_command = {}
        # Timestamped ring of recent commands per joint, so the external torque
        # estimate can reconstruct the setpoint the firmware is currently acting
        # on rather than the newest one (see _delayed_command). Sized to cover
        # the configured delay with margin at the control rate.
        self.command_history = {}
        self._command_history_len = max(
            4, int(self.params.firmware_command_delay * self.params.control_rate) + 3
        )

        self.external_torque_publisher = None
        self.momentum_observer = None
        self.residual_filter = None
        if self.params.external_torque_estimation.enable:
            if self.gravity_model is None:
                raise RuntimeError(
                    "external_torque_estimation.enable is true but no gravity model was "
                    "loaded (gravity_compensation.model_path is empty). Set it to a "
                    "MuJoCo .xml or a .urdf/.xacro model file."
                )
            if self.params.external_torque_estimation.method == "momentum_observer":
                from agx_arm_pd_g_controller.gravity_compensation import GravityCompensationModel
                from agx_arm_pd_g_controller.momentum_observer import MomentumObserver

                # The momentum observer reconstructs the full dynamics from the
                # mass matrix and bias force, which are constraint-unaware, so it
                # needs a model containing only the controlled joints - load a
                # separate instance with any other joints (e.g. a gripper's,
                # possibly linked by an equality constraint) welded shut, rather
                # than reusing gravity_model's (unwelded) model.
                momentum_observer_model = GravityCompensationModel(
                    model_path=self.params.gravity_compensation.model_path,
                    joint_names=self.params.joints,
                    weld_joints_except=self.params.joints,
                )
                self.momentum_observer = MomentumObserver(
                    model=momentum_observer_model.model,
                    qpos_indices=momentum_observer_model.qpos_indices,
                    qvel_indices=momentum_observer_model.qvel_indices,
                    gain=self.params.external_torque_estimation.momentum_observer_gain,
                )
            if self.params.external_torque_estimation.residual_filter_hz > 0.0:
                from agx_arm_pd_g_controller.residual_filter import LowPassFilter

                cutoff = self.params.external_torque_estimation.residual_filter_hz
                self.residual_filter = LowPassFilter(
                    cutoff_hz=cutoff,
                    sample_rate_hz=self.params.control_rate,
                    n_channels=len(self.params.joints),
                )
                self.get_logger().info(
                    f"External torque estimate low-passed at {cutoff:.1f} Hz "
                    f"(gain {self.residual_filter.gain_at(15.5):.3f} at 15.5 Hz, "
                    f"{self.residual_filter.gain_at(1.0):.3f} at 1 Hz)"
                )
            self.external_torque_publisher = self.create_publisher(
                JointState, self.params.external_torque_estimation.output_topic, 1
            )
            self.external_torque_timer = self.create_timer(
                1.0 / self.params.control_rate, self._publish_external_torque
            )
            self.get_logger().info(
                f"External torque estimation ('{self.params.external_torque_estimation.method}') "
                f"-> '{self.params.external_torque_estimation.output_topic}'"
            )

        self.publisher = self.create_publisher(MoveMITMsg, self.params.output_topic, 1)
        self.feedback_subscription = self.create_subscription(
            JointState, self.params.feedback_topic, self._feedback_callback, 1
        )
        self.subscription = self.create_subscription(
            JointState, self.params.input_topic, self._joint_state_callback, 1
        )
        self.control_timer = self.create_timer(
            1.0 / self.params.control_rate, self._control_step
        )
        self.get_logger().info(
            f"PD{'+G' if self.params.gravity_compensation.enable else ''} control: "
            f"'{self.params.input_topic}' -> '{self.params.output_topic}' "
            f"for joints {self.params.joints} (feedback: '{self.params.feedback_topic}') "
            f"at {self.params.control_rate} Hz"
        )

    def _feedback_callback(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.measured_positions[name] = position
        if len(msg.velocity) == len(msg.name):
            for name, velocity in zip(msg.name, msg.velocity):
                self.measured_velocities[name] = velocity
        if len(msg.effort) == len(msg.name):
            for name, effort in zip(msg.name, msg.effort):
                self.measured_efforts[name] = effort

    def _delayed_command(self, name, now):
        """The MIT command the firmware is actually acting on right now.

        Commands take firmware_command_delay to reach the motor, so at wall time
        `now` the firmware is still tracking the setpoint published
        `firmware_command_delay` ago - while using its own *current* encoder
        reading. Reconstructing with the newest setpoint instead pairs a command
        with a state it never saw, which the fit sees as an apparent gain error;
        modelling the delay here is what lets firmware_gain_scaling hold the
        arm's true multiplier rather than one distorted to absorb the mismatch.

        Linearly interpolates between the two commands bracketing the target
        time, since the delay is not an integer number of control periods.
        """
        history = self.command_history.get(name)
        if not history:
            return None
        target = now - self.params.firmware_command_delay
        if target <= history[0][0]:
            return history[0][1]
        if target >= history[-1][0]:
            return history[-1][1]
        # Walk back to the newest entry at or before the target, so the target
        # is bracketed by [k, k+1]. Stopping at the newest pair instead would
        # extrapolate backwards from it, which amplifies command noise rather
        # than interpolating.
        for k in range(len(history) - 2, -1, -1):
            t0, older = history[k]
            if t0 > target:
                continue
            t1, newer = history[k + 1]
            if t1 <= t0:
                return newer
            alpha = (target - t0) / (t1 - t0)
            return {
                # kp/kd are configuration, not a trajectory - do not interpolate
                "kp": newer["kp"],
                "kd": newer["kd"],
                "p_des": older["p_des"] + alpha * (newer["p_des"] - older["p_des"]),
                "v_des": older["v_des"] + alpha * (newer["v_des"] - older["v_des"]),
                "torque": older["torque"] + alpha * (newer["torque"] - older["torque"]),
            }
        return history[0][1]

    def _commanded_torques(self, qpos, qvel):
        """Best available reconstruction of the total actuator torque, per joint.

        Evaluates the firmware's own tracking law (kp*(p_des-p)+kd*(v_des-v)+t_ff)
        so the arm's own tracking effort isn't misread as external torque, using
        the setpoint the firmware is actually acting on (see _delayed_command)
        against the current measured state. gains.kp/kd are scaled by
        firmware_gain_scaling first, since the firmware applies its own internal
        multiplier to them. Falls back to gravity alone before the first command
        has been sent for a joint (e.g. before the first setpoint, or during
        move_j-based homing).
        """
        gravity = self.gravity_model.raw_gravity_torque(qpos)
        now = self.get_clock().now().nanoseconds * 1e-9
        commanded = []
        for i, name in enumerate(self.params.joints):
            command = self._delayed_command(name, now)
            if command is None:
                commanded.append(gravity[i])
            else:
                scale = self.joint_list_index[name]
                kp = command["kp"] * self.params.firmware_gain_scaling.kp[scale]
                kd = command["kd"] * self.params.firmware_gain_scaling.kd[scale]
                commanded.append(
                    kp * (command["p_des"] - qpos[i])
                    + kd * (command["v_des"] - qvel[i])
                    + command["torque"]
                )
        return commanded

    def _external_torques(self):
        """Per-joint external torque estimate, or None if not ready.

        measured_effort is the *total* motor torque, which includes whatever
        the MIT firmware's own tracking law is actively commanding to chase the
        last-sent reference - not just gravity and contact. With method
        'commanded_torque_diff', that reconstructed commanded torque is simply
        subtracted from measured_effort; this is cheap but, during motion,
        conflates genuine inertial/Coriolis torque (needed to actually move the
        arm) with external contact torque, since it has no dynamic model of the
        arm's own motion. With method 'momentum_observer', the same commanded
        torque instead drives a De Luca generalized-momentum residual filter
        (see momentum_observer.py) that correctly separates inertial/Coriolis
        torque from external torque using MuJoCo's mass matrix and bias force -
        fed either that reconstruction or measured_effort itself, per
        external_torque_estimation.torque_source.
        """
        try:
            qpos = [self.measured_positions[name] for name in self.params.joints]
            qvel = [self.measured_velocities.get(name, 0.0) for name in self.params.joints]
            measured = [self.measured_efforts[name] for name in self.params.joints]
        except KeyError:
            return None

        commanded = self._commanded_torques(qpos, qvel)

        if self.momentum_observer is not None:
            # The observer wants the torque the actuator actually applied. The
            # reconstruction is only a proxy for it, and a poor one on joints
            # 1-3: the firmware applies its own multiplier to kp/kd (~25x, see
            # models/README.md), so with
            # firmware_gain_scaling at 1.0 the reconstruction understates them
            # badly. 'measured' sidesteps the whole question.
            torque = (
                measured
                if self.params.external_torque_estimation.torque_source == "measured"
                else commanded
            )
            return list(
                self.momentum_observer.update(
                    qpos, qvel, torque, 1.0 / self.params.control_rate
                )
            )

        return [m - c for m, c in zip(measured, commanded)]

    def _publish_external_torque(self) -> None:
        tau_ext = self._external_torques()
        if tau_ext is None:
            return
        if self.residual_filter is not None:
            # Filter here rather than inside the observer: this way both
            # estimation methods get it, and the observer's own residual stays
            # untouched for anyone reading it directly.
            tau_ext = list(self.residual_filter(tau_ext))
        msg = JointState()
        msg.name = list(self.params.joints)
        msg.position = [self.measured_positions[name] for name in self.params.joints]
        msg.effort = list(tau_ext)
        self.external_torque_publisher.publish(msg)

    def _gravity_torques(self):
        """Per-joint gravity torque dict, or None if it cannot be computed yet."""
        try:
            qpos = [self.measured_positions[name] for name in self.params.joints]
        except KeyError as e:
            self.get_logger().warning(
                f"Gravity compensation enabled but no feedback received yet for joint "
                f"{e} on '{self.params.feedback_topic}'. Not publishing commands.",
                throttle_duration_sec=5.0,
            )
            return None
        tau = self.gravity_model.predict(qpos)
        return dict(zip(self.params.joints, tau))

    def _joint_state_callback(self, msg: JointState) -> None:
        self.latest_setpoint = msg

    def _control_step(self) -> None:
        msg = self.latest_setpoint
        if msg is None:
            return

        if self.param_listener.is_old(self.params):
            self.params = self.param_listener.get_params()

        gravity_torques = None
        if self.params.gravity_compensation.enable:
            if self.gravity_model is None:
                self.get_logger().error(
                    "gravity_compensation.enable is true but no model was loaded at "
                    "startup (gravity_compensation.model_path was empty). "
                    "Not publishing commands.",
                    throttle_duration_sec=5.0,
                )
                return
            gravity_torques = self._gravity_torques()
            if gravity_torques is None:
                return

        has_velocity = len(msg.velocity) == len(msg.name)
        has_effort = len(msg.effort) == len(msg.name)
        now = self.get_clock().now().nanoseconds * 1e-9

        out = MoveMITMsg()
        for i, name in enumerate(msg.name):
            joint_index = self.joint_indices.get(name)
            if joint_index is None:
                continue

            gains = self.params.gains.get_entry(name)
            torque = gains.feedforward_torque
            if gravity_torques is not None:
                torque += gravity_torques[name]
            if self.params.remote_force_feedforward.enable and has_effort:
                scale = self.params.remote_force_feedforward.torque_scaling[
                    self.joint_list_index[name]
                ]
                torque += msg.effort[i] * scale

            p_des = msg.position[i]
            v_des = msg.velocity[i] if has_velocity else 0.0

            out.joint_index.append(joint_index)
            out.p_des.append(p_des)
            out.v_des.append(v_des)
            out.kp.append(gains.kp)
            out.kd.append(gains.kd)
            out.torque.append(torque)

            command = {
                "p_des": p_des,
                "v_des": v_des,
                "kp": gains.kp,
                "kd": gains.kd,
                "torque": torque,
            }
            self.last_command[name] = command
            history = self.command_history.setdefault(name, deque(maxlen=self._command_history_len))
            history.append((now, command))

        if not out.joint_index:
            return

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PdGControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
