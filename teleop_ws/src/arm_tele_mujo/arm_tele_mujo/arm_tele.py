#! /usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Int32MultiArray


import mujoco
import mujoco.viewer
import numpy as np
import time
import threading
from pynput import keyboard

from .mjmodel import mj_armHandSys
from .drake_model import arm_ik,Quatnumpy_to_Rotation


class ArmTele(Node):
    def __init__(self,):
        super().__init__('arm_tele')
        self.get_logger().info('Arm Teleop Node Started')
        self.quat_sub = self.create_subscription(
            Float32MultiArray,
            'quat_data', self.quat_callback, 10)

        self.package_name = 'arm_hand_description'

        self.declare_parameter('model_name', 'iiwa14_7dof')
        self.model_name = self.get_parameter('model_name').value

        self.robot = mj_armHandSys(self.model_name,self.package_name)
        self.get_logger().info(f'Loading MuJoCo model: {self.robot.mjcf_model_path}')

        self._ik = arm_ik(f'{self.robot.sdf_model}',self.model_name)
        if  '_7dof' in self.model_name:
            self._q = np.array([0.2, 0.4, 0.6, 1.5, -0.3, -1.0,0.7])
        elif '_6dof' in self.model_name:
            self._q = np.array([0.2, 0.4, 0.6, 0.5, -0.3,0]) 

        

    def quat_callback(self, msg):
        now = int(time.time())

        self.quats = np.array(msg.data) # 50HZ
        rots = Quatnumpy_to_Rotation(self.quats)
        self._q = self._ik.ori_inv(up_ori=rots[2].matrix(),elbow_ori=rots[1].matrix(),wrist_ori=rots[0].matrix(),q_last=self._q)



def main(args=None):

    rclpy.init(args=args)
    arm_tele = ArmTele()

    try:
        rclpy.spin(arm_tele)  # 运行ROS事件循环
    except KeyboardInterrupt:
        pass

    arm_tele.destroy_node()
    rclpy.shutdown()