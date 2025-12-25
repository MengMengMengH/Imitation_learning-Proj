#! /usr/bin/env python3
import rclpy
from std_msgs.msg import Float32MultiArray
# from rclpy.executors import MultiThreadedExecutor
from cust_msgs.msg import Stampfloat32array

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import rclpy.duration

import numpy as np

from .drake_model import Quatnumpy_to_Rotation
from .arm_tele import ArmTele

class ArmTele_real(ArmTele):
    def __init__(self):

        super().__init__()
        self.get_logger().info('ArmTele_real Node Started')
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            deadline=rclpy.duration.Duration(seconds=0, nanoseconds=1000000)
        )

        self.received_first_msg = False
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.zero_joints = np.zeros(7, dtype=np.float32)

        self.control_pub_ = self.create_publisher(
            Float32MultiArray, '/rokae_control_joints', qos_profile)
        self.origin_data_pub_ = self.create_publisher(
            Stampfloat32array, 'origin_quat_data_used', 10)
        self.last_valid_q = self._q

        # if self._q is not None:
        #     self.real_q = self._q.copy()

    def timer_callback(self):
        if not self.received_first_msg:
            goal_msg = Float32MultiArray()
            goal_msg.data = self.zero_joints.tolist()
            self.control_pub_.publish(goal_msg)


    def quat_callback(self, msg):

        if not self.received_first_msg:
                self.received_first_msg = True
                self.timer.cancel()
                self.timer.destroy()
                self.get_logger().info('Received first valid message, stopped zero-joint timer.')

        self.quats = np.array(msg.data)
        rots = Quatnumpy_to_Rotation(self.quats) 
        # print(f"last value:{self.last_valid_q}")

        self._q = self._ik.ori_inv(
            up_ori=rots[2].matrix(),
            elbow_ori=rots[1].matrix(),
            wrist_ori=rots[0].matrix(),
            q_last=self._q
        )
        if self._q is not None:
            self.last_valid_q = self._q
            goal_msg = Float32MultiArray()
            goal_msg.data = self._q.tolist()
            self.control_pub_.publish(goal_msg)
            ori_data_msg = Stampfloat32array()
            ori_data_msg.header.stamp = self.get_clock().now().to_msg()
            ori_data_msg.data = self.quats[0:12].tolist()
            self.origin_data_pub_.publish(ori_data_msg)
        else:
            self.get_logger().warn('No valid joint angles found,sent last valid joint angles')
            goal_msg = Float32MultiArray()
            goal_msg.data = self.last_valid_q.tolist()
            self.control_pub_.publish(goal_msg)
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

