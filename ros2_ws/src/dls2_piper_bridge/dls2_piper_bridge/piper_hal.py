import rclpy
from rclpy.node import Node
from dls2_interface.msg import ArmState, ArmTrajectoryGenerator, ArmControlSignal

import numpy as np
import time

from pyAgxArm import AgxArmFactory, ArmModel, create_agx_arm_config


class PiperHALNode(Node):
    def __init__(self):
        super().__init__('Piper_HAL_Node')

        self.declare_parameter("can_port", "can0")
        self.can_port = self.get_parameter("can_port").value

        arm_state_freq = 300  # Hz
        self.timer = self.create_timer(1/arm_state_freq, self.compute_piper_hal_callback)
        self.publisher_arm_blind_state = self.create_publisher(ArmState, "/arm_state", 1)
        self.subscriber_trajectory_generator_arm = self.create_subscription(
            ArmTrajectoryGenerator, "/arm_trajectory_generator", self.get_arm_trajectory_generator_callback, 1)
        self.subscriber_arm_control_signal = self.create_subscription(
            ArmControlSignal, "/arm_control_signal", self.get_arm_control_signal_callback, 1)

        # Initialize control signal variables
        self.desired_arm_joints_torque = np.zeros(6)
        self.desired_gripper_torque = 0.0

        # Initialize state variables
        self.arm_state_sequence_id = 0
        self.arm_joints_name = [f"joint_{i}" for i in range(1, 7)]
        self.previous_arm_state_timestamp = None
        self.previous_arm_joints_velocity = np.zeros(6)
        self.previous_gripper_position = 0.0
        self.current_gripper_position = 0.0
        self.current_gripper_effort = 0.0

        np.set_printoptions(precision=3, suppress=True)
        piper_config = create_agx_arm_config(
            robot=ArmModel.PIPER,
            comm="can",
            channel=self.can_port,
        )
        self.piper = AgxArmFactory.create_arm(piper_config)
        self.piper.connect()
        self.gripper = self.piper.init_effector(self.piper.OPTIONS.EFFECTOR.AGX_GRIPPER)
        self.piper.set_motion_mode(self.piper.OPTIONS.MOTION_MODE.MIT)

        while not self.piper.enable():
            self.get_logger().info("Enabling Piper...")
            time.sleep(0.01)
        self.get_logger().info("Piper enabled.")


    def get_arm_trajectory_generator_callback(self, msg):
        desired_arm_joints_position = np.array(msg.desired_arm_joints_position)
        desired_arm_joints_velocity = np.array(msg.desired_arm_joints_velocity)

        kp = np.array(msg.arm_kp)
        kd = np.array(msg.arm_kd)

        # Arm control
        for i in range(6):
            self.piper.move_mit(
                i + 1,
                p_des=desired_arm_joints_position[i],
                v_des=desired_arm_joints_velocity[i],
                kp=kp[i],
                kd=kd[i],
                t_ff=self.desired_arm_joints_torque[i],
            )

        # Gripper control
        self.gripper.move_gripper_m(
            value=float(msg.desired_arm_gripper_position),
            force=max(0.0, float(self.desired_gripper_torque)),
        )


    def get_arm_control_signal_callback(self, msg):
        self.desired_arm_joints_torque = np.array(msg.desired_arm_joints_torque)
        self.desired_gripper_torque = msg.desired_arm_gripper_torque


    def compute_piper_hal_callback(self):
        timestamp = self.get_clock().now().nanoseconds * 1e-9
        motor_states = [self.piper.get_motor_states(i) for i in range(1, 7)]
        if any(motor_state is None for motor_state in motor_states):
            return

        driver_states = [self.piper.get_driver_states(i) for i in range(1, 7)]

        joints_position = np.array(
            [motor_state.msg.position for motor_state in motor_states],
            dtype=float,
        )
        joints_velocity = np.array(
            [motor_state.msg.velocity for motor_state in motor_states],
            dtype=float,
        )
        joints_effort = np.array(
            [motor_state.msg.torque for motor_state in motor_states],
            dtype=float,
        )
        joints_temperature = np.array(
            [
                driver_state.msg.motor_temp if driver_state is not None else 0.0
                for driver_state in driver_states
            ],
            dtype=float,
        )

        gripper_status = self.gripper.get_gripper_status()
        if gripper_status is not None:
            self.current_gripper_position = gripper_status.msg.value
            self.current_gripper_effort = gripper_status.msg.force

        gripper_position = self.current_gripper_position
        gripper_effort = self.current_gripper_effort

        joints_acceleration = np.zeros(6)
        gripper_velocity = 0.0
        if self.previous_arm_state_timestamp is not None:
            dt = timestamp - self.previous_arm_state_timestamp
            if dt > 0.0:
                joints_acceleration = (joints_velocity - self.previous_arm_joints_velocity) / dt
                gripper_velocity = (gripper_position - self.previous_gripper_position) / dt

        arm_state_msg = ArmState()
        arm_state_msg.frame_id = "base_link"
        arm_state_msg.sequence_id = self.arm_state_sequence_id
        arm_state_msg.timestamp = timestamp
        arm_state_msg.robot_name = "piper"
        arm_state_msg.joints_name = self.arm_joints_name
        arm_state_msg.joints_position = joints_position.tolist()
        arm_state_msg.joints_velocity = joints_velocity.tolist()
        arm_state_msg.joints_acceleration = joints_acceleration.tolist()
        arm_state_msg.joints_effort = joints_effort.tolist()
        arm_state_msg.joints_temperature = joints_temperature.tolist()
        arm_state_msg.gripper_position = float(gripper_position)
        arm_state_msg.gripper_velocity = float(gripper_velocity)
        arm_state_msg.gripper_effort = float(gripper_effort)

        self.publisher_arm_blind_state.publish(arm_state_msg)
        self.arm_state_sequence_id += 1
        self.previous_arm_state_timestamp = timestamp
        self.previous_arm_joints_velocity = joints_velocity
        self.previous_gripper_position = gripper_position

    def destroy_node(self):
        self.piper.disconnect()
        super().destroy_node()


def main(args=None):
    print('Hello from the Piper hal node.')
    
    rclpy.init(args=args)
    piper_hal_node = PiperHALNode()
    try:
        rclpy.spin(piper_hal_node)
    except KeyboardInterrupt:
        pass
    finally:
        piper_hal_node.destroy_node()
        rclpy.shutdown()
    
    print("Piper hal node is stopped")


if __name__ == '__main__':
    main()
