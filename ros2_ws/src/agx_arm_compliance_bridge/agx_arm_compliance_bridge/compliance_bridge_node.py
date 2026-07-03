import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from agx_arm_msgs.msg import MoveMITMsg

from agx_arm_compliance_bridge.agx_arm_compliance_bridge_parameters import compliance_bridge


class ComplianceBridgeNode(Node):
    """Republishes JointState commands as MoveMITMsg, adding per-joint MIT compliance gains."""

    def __init__(self):
        super().__init__("agx_arm_compliance_bridge")

        self.param_listener = compliance_bridge.ParamListener(self)
        self.params = self.param_listener.get_params()

        if not self.params.joints:
            raise RuntimeError(
                "Parameter 'joints' is empty. Set it in the node's YAML config to the "
                "ordered list of joint names to control."
            )
        # The Piper SDK's MIT joint_index is 1-based (joint1 == 1, ..., joint6 == 6).
        self.joint_indices = {name: i for i, name in enumerate(self.params.joints, start=1)}

        self.publisher = self.create_publisher(MoveMITMsg, self.params.output_topic, 1)
        self.subscription = self.create_subscription(
            JointState, self.params.input_topic, self._joint_state_callback, 1
        )
        self.get_logger().info(
            f"Bridging '{self.params.input_topic}' -> '{self.params.output_topic}' "
            f"for joints {self.params.joints}"
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        if self.param_listener.is_old(self.params):
            self.params = self.param_listener.get_params()

        has_velocity = len(msg.velocity) == len(msg.name)

        out = MoveMITMsg()
        for i, name in enumerate(msg.name):
            joint_index = self.joint_indices.get(name)
            if joint_index is None:
                continue

            gains = self.params.compliance.get_entry(name)

            out.joint_index.append(joint_index)
            out.p_des.append(msg.position[i])
            out.v_des.append(msg.velocity[i] if has_velocity else 0.0)
            out.kp.append(gains.kp)
            out.kd.append(gains.kd)
            out.torque.append(gains.feedforward_torque)

        if not out.joint_index:
            return

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ComplianceBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
