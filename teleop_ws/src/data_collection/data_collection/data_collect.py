#!/usr/bin/env python3
import os
import csv
import cv2
import h5py
import numpy as np
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from cust_msgs.msg import Stampfloat32array,Stampint32array
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters


class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        self.get_logger().info('Data Collector Node has been started.')

        # === 数据保存路径 ===
        save_root = os.path.expanduser("~/robot_dataset")
        os.makedirs(save_root, exist_ok=True)
        self.episode_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_dir = os.path.join(save_root, self.episode_name)
        os.makedirs(self.save_dir, exist_ok=True)
        os.makedirs(os.path.join(self.save_dir, "images"), exist_ok=True)

        self.bridge = CvBridge()
        # Initialize data collection mechanisms here
        self.sub_arm_states_ = message_filters.Subscriber(self,
            Stampfloat32array,
            'origin_quat_data_used',
            # self.listener_callback,
            # 10,
            )
        self.sub_hand_states = message_filters.Subscriber(self,
            Stampint32array,
            'hand_states',
            # self.listener_callback,
            # 10,
            )
        self.sub_force_data = message_filters.Subscriber(self,
            Stampfloat32array,
            'force_data',
            # self.listener_callback,
            # 10,
            )
        self.sub_image_data = message_filters.Subscriber(self,
            Image,
            'wrist_camera_Image',
            # self.listener_callback,
            # 10,
            )
        
        ats = message_filters.ApproximateTimeSynchronizer(
            [self.sub_arm_states_,self.sub_hand_states,self.sub_force_data,self.sub_image_data],
            queue_size=10,
            slop=0.01)
        
        ats.registerCallback(self.sync_callback)
        self.timestamps = []
        self.arm_states = []
        self.hand_states = []
        self.force_data = []
        self.image_paths = []
        self.frame_count = 0

        self.get_logger().info("Data synchronization node started!")

    def sync_callback(self, arm_states, hand_states, force_data, image_data):
        # 处理同步后的消息
        timestamp = image_data.header.stamp.sec + image_data.header.stamp.nanosec * 1e-9
        
        # ---- 图像保存 ----
        cv_image = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='bgr8')
        image_filename = f"img_{self.frame_count:06d}.png"
        img_path = os.path.join(self.save_dir, "images", image_filename)
        cv2.imwrite(img_path, cv_image)

        # ---- 缓存数据 ----
        self.timestamps.append(timestamp)
        self.image_paths.append(img_path)
        self.arm_states.append(np.array(arm_states.data,dtype=np.float32))
        self.hand_states.append(np.array(hand_states.data,dtype=np.int32))
        self.force_data.append(np.array(force_data.data,dtype=np.float32))
        self.frame_count += 1

        if self.frame_count % 100 == 0:
            self.get_logger().info(f"Collected {self.frame_count} frames.")

    def save2csv(self):
        csv_path = os.path.join(self.save_dir, "data_log.csv")
        self.get_logger().info(f"💾 Writing CSV data to {csv_path}")
        with open(csv_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['timestamp', 'image_path', 'arm_states[]', 'hand_states[]', 'force_data'])
            for i, ts in enumerate(self.timestamps):
                writer.writerow([
                    ts,
                    self.image_paths[i],
                    ' '.join(map(str, self.arm_states[i])),
                    ' '.join(map(str, self.hand_states[i])),
                    ' '.join(map(str, self.force_data[i]))
                ])
        self.get_logger().info(f"Data saved to {csv_path}")
    
    def save2hdf5(self):
        h5_path = os.path.join(self.save_dir, "data_log.h5")
        self.get_logger().info(f"💾 Converting to HDF5: {h5_path}")

        imgs = [cv2.imread(p) for p in self.image_paths]
        imgs = np.stack(imgs)
        arm = np.stack(self.arm_states)
        hand = np.stack(self.hand_states)
        force = np.stack(self.force_data)
        ts = np.array(self.timestamps)

        with h5py.File(h5_path, "w") as f:
            f.create_dataset("images", data=imgs, compression="gzip")
            f.create_dataset("arm_states", data=arm)
            f.create_dataset("hand_states", data=hand)
            f.create_dataset("forces", data=force)
            f.create_dataset("timestamps", data=ts)
        self.get_logger().info("✅ HDF5 dataset saved successfully.")
    
    def stop_and_save(self):
        """任务结束时调用：保存所有文件"""
        self.get_logger().info("🧹 Stopping collector, saving all data...")
        if len(self.timestamps) == 0:
            self.get_logger().warn("⚠️ No data captured, skipping save.")
            return
        self.save2csv()
        self.save2hdf5()
        self.get_logger().info("🎉 All data saved successfully.")


def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down.')
    finally:
        # 清理节点和ROS 2环境
        node.stop_and_save()
        node.destroy_node()
        rclpy.shutdown()

