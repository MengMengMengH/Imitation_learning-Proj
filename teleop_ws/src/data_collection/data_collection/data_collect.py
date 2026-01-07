#!/usr/bin/env python3
import os
import csv
import cv2
import h5py
import numpy as np
from datetime import datetime
import threading
import queue

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
from cust_msgs.msg import Stampfloat32array,Stampint32array
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
from pydrake.all import (
    RotationMatrix,
    Quaternion,
    AngleAxis,
)

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        self.get_logger().info('Data Collector Node has been started.')

        # === 数据保存路径 ===
        save_root = os.path.expanduser("~/Imitation_learning-Proj/robot_dataset")
        os.makedirs(save_root, exist_ok=True)
        self.episode_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_dir = os.path.join(save_root, self.episode_name)
        os.makedirs(self.save_dir, exist_ok=True)
        self.img_save_dir = os.path.join(self.save_dir, "images")
        os.makedirs(self.img_save_dir, exist_ok=True)

        self.bridge = CvBridge()

        # === 数据缓存列表 ===
        self.timestamps = []
        self.arm_states = []
        self.hand_states = []
        self.force_data = []
        self.image_paths = []
        self.frame_count = 0

        # === 最新消息缓存 ===
        self.latest_cam_image = None
        self.latest_force_data = None

        # === 新鲜度检查 ===
        self.tolerances_ns = {
            'camera': 50_000_000,     # 50 ms
            'force': 20_000_000,      # 20 ms
        }

        # === 创建I/O线程和队列

        self.save_que = queue.Queue(maxsize=500)
        self.save_thread = threading.Thread(target=self.img_save_loop)
        self.save_thread_running = True
        self.save_thread.start()
        self.get_logger().info("✅ I/O 线程已启动，用于异步保存图像")

        # === QoS 配置 ===
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # === 订阅器 ===
        self.arm_states_sub = message_filters.Subscriber(
            self,
            Stampfloat32array,
            'origin_quat_data_used',
            qos_profile=qos_profile
        )
        self.hand_states_sub = message_filters.Subscriber(
            self,
            Stampint32array,
            'hand_states',
            qos_profile=qos_profile
        )

        self.force_sub = self.create_subscription(
            Stampfloat32array,
            'force_data',
            self.force_callback,
            qos_profile
        )
        self.cam_sub = self.create_subscription(
            Image,
            'wrist_camera_Image',
            self.cam_callback,
            qos_profile
        )

        # === 创建动作同步器(ATS) ===
        self.action_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.arm_states_sub, self.hand_states_sub],
            queue_size=10,
            slop=0.02 
        )
        self.action_synchronizer.registerCallback(self.main_trigger_callback)
        self.get_logger().info("✅ (Arm, Hand) 动作同步器已启动。")
        self.get_logger().info("...等待所有数据源的初始消息...")

    def img_save_loop(self):
        while self.save_thread_running:
            try:
                task = self.save_que.get(timeout=1)
                if task is None:
                    continue
                (img_path,img_data) = task

                cv_img = self.bridge.imgmsg_to_cv2(img_data,desired_encoding='bgr8')
                cv2.imwrite(img_path,cv_img,[int(cv2.IMWRITE_JPEG_QUALITY),95])
            except queue.Empty:
                pass
            except Exception as e:
                self.get_logger().error(f'图像线程保存出错:{e}')
        self.get_logger().info("图像保存线程已退出")
        
    # === 简单更新回调 ===
    def force_callback(self, msg):
        self.latest_force_data = msg
    def cam_callback(self, msg):
        self.latest_cam_image = msg

    # === 主触发回调 ===
    def main_trigger_callback(self, arm_msg,hand_msg):
        # 检查所有数据源是否都有最新消息
        if (self.latest_cam_image is None or 
            self.latest_force_data is None):
            self.get_logger().warn('仍在等待传感器的初始消息...', throttle_duration_sec=5.0)
            return
        try:
            t_anchor = rclpy.time.Time.from_msg(arm_msg.header.stamp)
        except AttributeError:
            self.get_logger().error(f'自定义消息中没有时间辍',once = True)
            return
        
        # 数据新鲜度检查
        t_camera = rclpy.time.Time.from_msg(self.latest_cam_image.header.stamp)
        dt_camera_ns = (t_anchor - t_camera).nanoseconds
        if dt_camera_ns < 0 or dt_camera_ns > self.tolerances_ns['camera']:
            self.get_logger().warn(f'⚠️ 丢弃帧：相机数据陈旧 (dt={dt_camera_ns/1e6:.1f}ms)')
            return

        t_force = rclpy.time.Time.from_msg(self.latest_force_data.header.stamp)
        dt_force_ns = (t_anchor - t_force).nanoseconds
        if dt_force_ns < 0 or dt_force_ns > self.tolerances_ns['force']:
            self.get_logger().warn(f'⚠️ 丢弃帧：力传感器数据陈旧 (dt={dt_force_ns/1e6:.1f}ms)')
            return

        
        #  === 检查通过 ===
        timestamp = t_anchor.seconds_nanoseconds()[0] + t_anchor.seconds_nanoseconds()[1] * 1e-9
        
        # ---- 图像数据 ----
        image_data = self.latest_cam_image
        # cv_image = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='bgr8')
        image_filename = f"img_{self.frame_count:06d}.jpg"
        img_path = os.path.join(self.save_dir, "images", image_filename)
        try:
            # self.save_que.put_nowait((img_path, cv_image))
            self.save_que.put_nowait((img_path, image_data))
        except queue.Full:
            self.get_logger().warn("I/O 队列已满！图像保存滞后，丢弃此帧。")
            return

        # ---- 缓存数据 ----
        self.timestamps.append(timestamp)
        self.image_paths.append(img_path)

        rots = []
        quats = np.array(arm_msg.data).reshape(-1, 4)
        for quat in quats:
            rot = Quaternion(quat).rotation()
            rots.append(rot[:,:2].flatten())
        arm_data = np.concatenate(rots) 

        self.arm_states.append(np.array(arm_data,dtype=np.float32))
        self.hand_states.append(np.array(hand_msg.data,dtype=np.int32))
        self.force_data.append(np.array(self.latest_force_data.data,dtype=np.float32))

        self.frame_count += 1

        if self.frame_count % 100 == 0:
            self.get_logger().info(f"Collected {self.frame_count} frames.")

    def save2hdf5(self):
        h5_path = os.path.join(self.save_dir, "episode_0000.hdf5")
        self.get_logger().info(f"💾 Converting to HDF5: {h5_path}")
        if not self.image_paths:
            self.get_logger().warn("⚠️ No images captured, skipping HDF5 save.")
            return
        try:
            sample_img = cv2.imread(self.image_paths[0])
            if sample_img is None:
                raise IOError(f"Failed to read sample image: {self.image_paths[0]}")
            H, W, C = sample_img.shape
            N = len(self.image_paths) # 总帧数
            dtype = sample_img.dtype
        except Exception as e:
            self.get_logger().error(f"无法读取样本图像！错误: {e}")
            return

        arm = np.stack(self.arm_states)
        hand = np.stack(self.hand_states)
        force = np.stack(self.force_data)
        # ts = np.array(self.timestamps)

        action = np.concatenate([arm, hand], axis=1)

        with h5py.File(h5_path, "w") as f:
            # ========== observations ==========
            obs = f.create_group("observations")

            img_dset = f.create_dataset(
                "images",
                shape=(N, H, W, C),
                dtype=dtype,
                chunks=(1, H, W, C), # 按单帧分块
                compression="gzip"
            )
            img_dset[0] = sample_img
            
            for i in range(1, N):
                img_path = self.image_paths[i]
                img_data = cv2.imread(img_path)
                if img_data is not None:
                    img_dset[i] = img_data
                else:
                    self.get_logger().warn(f"无法读取图像 {img_path}，跳过该帧！")
                
                if i % 100 == 0 or i == N - 1:
                    self.get_logger().info(f"  ... saving image {i+1}/{N}")

            # --- low-dim observations ---
            obs.create_dataset("arm_states", data=arm)
            obs.create_dataset("hand_states", data=hand)
            obs.create_dataset("force", data=force)

            # ========== actions ==========
            f.create_dataset("actions", data=action)

        self.get_logger().info("✅ HDF5 dataset saved successfully.")
    
    def stop_and_save(self):
        """任务结束时调用：保存所有文件"""
        self.get_logger().info("🧹 Stopping collector, saving all data...")
        self.get_logger().info("Stopping I/O threading,等待所有图像写入磁盘...")
        self.save_thread_running = False
        self.save_que.put(None)
        self.save_thread.join(timeout=10.0)
        if self.save_thread.is_alive():
             self.get_logger().error("I/O 线程未能及时停止！")
        else:
             self.get_logger().info("✅ I/O 线程已停止。所有图像均已写入磁盘。")

        if len(self.timestamps) == 0:
            self.get_logger().warn("⚠️ No data captured, skipping save.")
            return
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

