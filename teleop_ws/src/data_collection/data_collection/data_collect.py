#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from cust_msgs.msg import Stampfloat32array,Stampint32array
from sensor_msgs.msg import Image
import message_filters


class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        self.get_logger().info('Data Collector Node has been started.')
        # Initialize data collection mechanisms here
        self.sub_arm_states_ = message_filters.Subscriber(self,
            Stampfloat32array,
            'origin_quat_data_used',
            # self.listener_callback,
            10)
        self.sub_hand_states = message_filters.Subscriber(self,
            Stampint32array,
            'hand_states',
            # self.listener_callback,
            10)
        self.sub_force_data = message_filters.Subscriber(self,
            Stampfloat32array,
            'force_data',
            # self.listener_callback,
            10)
        self.sub_image_data = message_filters.Subscriber(self,
            Image,
            'wrist_camera_Image',
            # self.listener_callback,
            10)
        
        ats = message_filters.ApproximateTimeSynchronizer(
            [self.sub_arm_states_,self.sub_hand_states,self.sub_force_data,self.sub_image_data],
            queue_size=10,
            slop=0.01)
        
        ats.registerCallback(self.sync_callback)
        self.get_logger().info("Data synchronization node started!")

    def sync_callback(self, arm_states, hand_states, force_data, image_data):
        # 处理同步后的消息
        pass



def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down.')
    finally:
        # 清理节点和ROS 2环境
        node.destroy_node()
        rclpy.shutdown()

