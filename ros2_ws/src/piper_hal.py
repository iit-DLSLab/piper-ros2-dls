import rclpy 
from rclpy.node import Node 
from dls2_interface.msg import ArmState, ArmTrajectoryGenerator, ArmControlSignal

import numpy as np
import time

from piper_sdk import *


class PiperHALNode(Node):
    def __init__(self):
        super().__init__('Piper_HAL_Node')

        arm_state_freq = 300  # Hz 
        self.timer = self.create_timer(1/arm_state_freq, self.compute_piper_hal_callback)
        self.publisher_arm_blind_state = self.create_publisher(ArmState,"/arm_state", 1)
        self.subscriber_trajectory_generator_arm = self.create_subscription(ArmTrajectoryGenerator,"/arm_trajectory_generator", self.get_arm_trajectory_generator_callback, 1)
        self.subscriber_arm_control_signal = self.create_subscription(ArmControlSignal,"/arm_control_signal", self.get_arm_control_signal_callback, 1)

        # Initialize control signal variables
        self.desired_arm_joints_torque = np.zeros(6)
        self.desired_gripper_torque = 0.0

        # Initialize state variables
        self.arm_state_sequence_id = 0
        self.arm_joints_name = [f"joint_{i}" for i in range(1, 7)]
        self.previous_arm_state_timestamp = None
        self.previous_arm_joints_velocity = np.zeros(6)
        self.previous_gripper_position = 0.0

        np.set_printoptions(precision=3, suppress=True)
        self.piper = C_PiperInterface_V2("can0")
        self.piper.ConnectPort()
        while( not self.piper.EnablePiper()):
            print("Enabling Piper...")
            time.sleep(0.01)
        print("Piper Enabled.")


    def get_arm_trajectory_generator_callback(self, msg):
        
        print("TODO: implement trajectory generator callback")
        desired_arm_joints_position = np.array(msg.desired_arm_joints_position) 
        desired_arm_joints_velocity = np.array(msg.desired_arm_joints_velocity)

        kp = np.array(msg.arm_kp)
        kd = np.array(msg.arm_kd)

        # arm control
        for i in range(6):
           self.piper.JointMitCtrl(i,
                                    desired_arm_joints_position[i],
                                    desired_arm_joints_velocity[i], 
                                    kp[i],kd[i],
                                    self.desired_arm_joints_torque[i])

        # gripper control #TODO


    def get_arm_control_signal_callback(self, msg):

        self.desired_arm_joints_torque = np.array(msg.desired_arm_joints_torque)
        self.desired_gripper_torque = msg.desired_arm_gripper_torque


    def compute_piper_hal_callback(self):
        timestamp = self.get_clock().now().nanoseconds * 1e-9
        high_spd_msg = self.piper.GetArmHighSpdInfoMsgs()
        low_spd_msg = self.piper.GetArmLowSpdInfoMsgs()
        gripper_msg = self.piper.GetArmGripperMsgs()

        high_spd_motors = [
            high_spd_msg.motor_1,
            high_spd_msg.motor_2,
            high_spd_msg.motor_3,
            high_spd_msg.motor_4,
            high_spd_msg.motor_5,
            high_spd_msg.motor_6,
        ]
        low_spd_motors = [
            low_spd_msg.motor_1,
            low_spd_msg.motor_2,
            low_spd_msg.motor_3,
            low_spd_msg.motor_4,
            low_spd_msg.motor_5,
            low_spd_msg.motor_6,
        ]

        joints_position = np.array([motor.pos for motor in high_spd_motors]) * 1e-3
        joints_velocity = np.array([motor.motor_speed for motor in high_spd_motors]) * 1e-3
        joints_effort = np.array([motor.effort for motor in high_spd_motors]) * 1e-3
        joints_temperature = np.array([motor.motor_temp for motor in low_spd_motors], dtype=float)

        gripper_position = gripper_msg.gripper_state.grippers_angle * 1e-3
        gripper_effort = gripper_msg.gripper_state.grippers_effort * 1e-3

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

#---------------------------
if __name__ == '__main__':
    
    print('Hello from the Piper hal node.')
    
    rclpy.init()
    piper_hal_node = PiperHALNode()
    rclpy.spin(piper_hal_node)
    piper_hal_node.destroy_node()
    rclpy.shutdown()

    print("Piper hal node is stopped")
    exit(0)
