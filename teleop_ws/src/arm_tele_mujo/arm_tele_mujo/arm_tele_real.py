#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Int32MultiArray

import numpy as np
from pynput import keyboard
from joint_angles_interface.srv import JointAngles

from .mjmodel import mj_armHandSys
from .drake_model import arm_ik,Quatnumpy_to_Rotation
from .arm_tele import ArmTele

class ArmTele_real(ArmTele):
    def __init__(self):

        super().__init__()
        self.joint_pub = self.create_publisher(Float32MultiArray, 'joint_angles', 10)
        self.get_logger().info('ArmTele_real Node Started with joint publisher')

        self.joint_client = self.create_client(JointAngles, 'get_real_joint_angles')
        while not self.joint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        if self._q is not None:
            self.real_q = self._q.copy()

    def quat_callback(self, msg):

        self.quats = np.array(msg.data)
        rots = Quatnumpy_to_Rotation(self.quats) 


        self._q = self._ik.ori_inv(
            up_ori=rots[2].matrix(),
            elbow_ori=rots[1].matrix(),
            wrist_ori=rots[0].matrix(),
            q_last=self._q
        )


        request = JointAngles.Request()
        if self._q is not None:
            request.joint_angles = self._q.tolist()


        future = self.joint_client.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            real_joint_angles = np.array(response.real_joint_angles)
            self.get_logger().info(f'Received real joint angles: {real_joint_angles}')

            if real_joint_angles.shape[0] == 7:
                self.real_q = real_joint_angles
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):

    rclpy.init(args=args)
    arm_tele = ArmTele_real()

    try:
        rclpy.spin(arm_tele)  # 运行ROS事件循环
    except KeyboardInterrupt:
        pass

    arm_tele.destroy_node()
    rclpy.shutdown()
