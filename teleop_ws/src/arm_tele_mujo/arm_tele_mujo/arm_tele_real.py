#! /usr/bin/env python3
import rclpy
from std_msgs.msg import Float32MultiArray
from rclpy.executors import MultiThreadedExecutor

import numpy as np

from .drake_model import Quatnumpy_to_Rotation
from .arm_tele import ArmTele

class ArmTele_real(ArmTele):
    def __init__(self):

        super().__init__()
        self.get_logger().info('ArmTele_real Node Started')
        self.control_pub_ = self.create_publisher(
            Float32MultiArray, '/rokae_control_joints', 10)
        self.origin_data_pub_ = self.create_publisher(
            Float32MultiArray, 'origin_quat_data_used', 10)

        # if self._q is not None:
        #     self.real_q = self._q.copy()

    def quat_callback(self, msg):

        self.quats = np.array(msg.data)
        rots = Quatnumpy_to_Rotation(self.quats) 

        self._q = self._ik.ori_inv(
            up_ori=rots[2].matrix(),
            elbow_ori=rots[1].matrix(),
            wrist_ori=rots[0].matrix(),
            q_last=self._q
            # q_last=self.real_q
        )

        # print(self._q)
        goal_msg = Float32MultiArray()
        ori_data_msg = Float32MultiArray()
        if self._q is not None:
            goal_msg.data = self._q.tolist()
            self.control_pub_.publish(goal_msg)

            ori_data_msg.data = self.quats[0:12].tolist()

            self.origin_data_pub_.publish(msg)
        else:
            self.get_logger().warn('No valid joint angles found.')
            return
        
def main(args=None):

    rclpy.init(args=args)
    arm_tele = ArmTele_real()
    try:
        rclpy.spin(arm_tele)
    except KeyboardInterrupt:
        arm_tele.get_logger().info('Keyboard interrupt received, shutting down.')
    finally:
        # 清理节点和ROS 2环境
        arm_tele.destroy_node()
        rclpy.shutdown()

