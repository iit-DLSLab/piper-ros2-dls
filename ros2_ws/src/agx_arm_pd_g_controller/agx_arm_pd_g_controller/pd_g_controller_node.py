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
                calibration_path=self.params.gravity_compensation.calibration_path or None,
                torque_scaling=self.params.gravity_compensation.torque_scaling,
            )
            self.get_logger().info(
                f"Gravity model loaded from '{self.params.gravity_compensation.model_path}' "
                + (
                    f"with calibration '{self.params.gravity_compensation.calibration_path}'"
                    if self.gravity_model.calibrated
                    else f"with direct torque scaling {list(self.params.gravity_compensation.torque_scaling)}"
                )
            )

        self.measured_positions = {}
        self.measured_efforts = {}
        self.latest_setpoint = None

        self.external_torque_publisher = None
        if self.params.external_torque_estimation.enable:
            if self.gravity_model is None:
                raise RuntimeError(
                    "external_torque_estimation.enable is true but no gravity model was "
                    "loaded (gravity_compensation.model_path is empty). Set it to a "
                    "MuJoCo .xml or a .urdf/.xacro model file."
                )
            self.external_torque_publisher = self.create_publisher(
                JointState, self.params.external_torque_estimation.output_topic, 1
            )
            self.external_torque_timer = self.create_timer(
                1.0 / self.params.control_rate, self._publish_external_torque
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
        if len(msg.effort) == len(msg.name):
            for name, effort in zip(msg.name, msg.effort):
                self.measured_efforts[name] = effort

    def _external_torques(self):
        """Per-joint external torque estimate (measured - MuJoCo gravity), or None if not ready."""
        try:
            qpos = [self.measured_positions[name] for name in self.params.joints]
            measured = [self.measured_efforts[name] for name in self.params.joints]
        except KeyError:
            return None
        gravity = self.gravity_model.raw_gravity_torque(qpos)
        return [m - g for m, g in zip(measured, gravity)]

    def _publish_external_torque(self) -> None:
        tau_ext = self._external_torques()
        if tau_ext is None:
            return
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

            out.joint_index.append(joint_index)
            out.p_des.append(msg.position[i])
            out.v_des.append(msg.velocity[i] if has_velocity else 0.0)
            out.kp.append(gains.kp)
            out.kd.append(gains.kd)
            out.torque.append(torque)

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
