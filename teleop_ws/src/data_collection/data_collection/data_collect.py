#!/usr/bin/env python3
import os
import csv
import cv2
import h5py
import numpy as np
from datetime import datetime
import threading
import queue
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
from cust_msgs.msg import Stampfloat32array,Stampint32array
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from concurrent.futures import ThreadPoolExecutor
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


        # === 性能压测变量 ===
        self.target_fps = 30.0
        self.start_time = None
        
        # 详细丢帧计数器
        self.drop_cam_stale = 0
        self.drop_force_stale = 0
        self.drop_queue_full = 0

        # === 最新消息缓存 ===
        self.latest_cam_image = None
        self.latest_force_data = None

        # === 新鲜度检查 ===
        self.tolerances_ns = {
            'camera': 50_000_000,     # 50 ms
            'force': 20_000_000,      # 20 ms
        }

        # === 创建I/O线程和队列

        self.save_pool = ThreadPoolExecutor(max_workers=4)
        self.get_logger().info("✅ 4线程并行 I/O 线程池已启动")


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

    def _async_save_image_task(self, img_path, img_data):
            """🌟 新增：独立的并行图像保存任务"""
            try:
                cv_img = self.bridge.imgmsg_to_cv2(img_data, desired_encoding='bgr8')
                cv2.imwrite(img_path, cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
            except Exception as e:
                # 退出时 logger 失效，使用 print 兜底
                print(f'❌ 图像保存出错 ({img_path}): {e}')

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
        if abs(dt_camera_ns) > self.tolerances_ns['camera']:
            self.drop_cam_stale += 1
            self.get_logger().warn(f'⚠️ 丢弃帧：相机数据陈旧 (dt={dt_camera_ns/1e6:.1f}ms)',throttle_duration_sec=1.0)
            return

        t_force = rclpy.time.Time.from_msg(self.latest_force_data.header.stamp)
        dt_force_ns = (t_anchor - t_force).nanoseconds
        if abs(dt_force_ns) > self.tolerances_ns['force']:
            self.drop_force_stale += 1
            self.get_logger().warn(f'⚠️ 丢弃帧：力传感器数据陈旧 (dt={dt_force_ns/1e6:.1f}ms)',throttle_duration_sec=1.0)
            return

        if self.start_time is None:
            self.start_time = time.monotonic()
            self.get_logger().info("⏱️ 成功接收首个完美对齐帧，开始压测计时！")

        #  === 检查通过 ===
        timestamp = t_anchor.seconds_nanoseconds()[0] + t_anchor.seconds_nanoseconds()[1] * 1e-9
        
        # ---- 图像数据 ----
        image_data = self.latest_cam_image
        # cv_image = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='bgr8')
        image_filename = f"img_{self.frame_count:06d}.jpg"
        img_path = os.path.join(self.save_dir, "images", image_filename)

        if self.save_pool._work_queue.qsize() > 500:
            self.drop_queue_full += 1
            self.get_logger().warn("I/O 线程池满载！硬盘跟不上，丢弃此帧。", throttle_duration_sec=1.0)
            return
        else:
            self.save_pool.submit(self._async_save_image_task, img_path, image_data)

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
            print(f"无法读取样本图像！错误: {e}")
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
                    print(f"无法读取图像 {img_path}，跳过该帧！")
                
                if i % 100 == 0 or i == N - 1:
                    print(f"  ... saving image {i+1}/{N}")

            # --- low-dim observations ---
            obs.create_dataset("arm_states", data=arm)
            obs.create_dataset("hand_states", data=hand)
            obs.create_dataset("force", data=force)

            # ========== actions ==========
            f.create_dataset("actions", data=action)

        self.get_logger().info("✅ HDF5 dataset saved successfully.")
    
    def stop_and_save(self):
            """保存所有文件 (🌟 全部替换为 print)"""
            self.gen_stress_report()
            print("\n🧹 Stopping collector, saving all data...")
            print("⏳ 正在停止并行 I/O 线程池，等待所有缓存图像榨干 CPU 存入硬盘...")
            
            # 🌟 修复：shutdown(wait=True) 会阻塞在这里，直到池子里所有图像全部完美存完！
            self.save_pool.shutdown(wait=True)
            print("✅ I/O 线程池已安全退出。所有图像均已写入磁盘。")

            if len(self.timestamps) == 0:
                print("⚠️ No data captured, skipping save.")
                return
                
            self.save2hdf5()
            print("🎉 All data saved successfully.")


    def gen_stress_report(self):
            """生成压测报告"""
            end_time = time.monotonic()
            if self.start_time is not None:
                actual_duration = end_time - self.start_time
                theoretical_frames = actual_duration * self.target_fps
                
                retention_rate = 0.0
                if theoretical_frames > 0:
                    retention_rate = (self.frame_count / theoretical_frames) * 100.0

                print("\n==================================================")
                print("📊 系统高通量采集压测报告 (Throughput & Persistence)")
                print("==================================================")
                print(f"⏱️  物理持续时间 : {actual_duration:.2f} 秒")
                print(f"🎯  目标采集频率 : {self.target_fps} Hz")
                print(f"📈  理论应存帧数 : {int(theoretical_frames)} 帧")
                print(f"✅  实际落盘帧数 : {self.frame_count} 帧")
                print(f"🏆  最终有效留存率: {retention_rate:.2f}%")
                print("--------------------------------------------------")
                print("🔍 丢包溯源详情 (Packet Loss Breakdown):")
                print(f"   ❌ 相机陈旧丢帧  : {self.drop_cam_stale}")
                print(f"   ❌ 力觉陈旧丢帧  : {self.drop_force_stale}")
                print(f"   ❌ I/O 队列满载  : {self.drop_queue_full}  <-- 证明并行队列有效性的关键！")
                print("==================================================")

                csv_filename = os.path.join(self.save_dir, "stress_test_report.csv")

                report_data = {
                                "Metric": "Value", 
                                "Actual_Duration_Seconds": f"{actual_duration:.4f}",
                                "Target_FPS_Hz": f"{self.target_fps:.1f}",
                                "Theoretical_Frames": str(int(theoretical_frames)),
                                "Actual_Saved_Frames": str(self.frame_count),
                                "Retention_Rate_Percent": f"{retention_rate:.4f}",
                                "Drop_Camera_Stale": str(self.drop_cam_stale),
                                "Drop_Force_Stale": str(self.drop_force_stale),
                                "Drop_IO_Queue_Full": str(self.drop_queue_full)
                            }
                try:
                    with open(csv_filename, mode='w', newline='', encoding='utf-8') as file:
                        writer = csv.writer(file)
                        for key, value in report_data.items():
                            writer.writerow([key, value])
                    print(f"💾 压测报告已成功保存至: {csv_filename}")
                except Exception as e:
                    print(f"❌ 保存 CSV 压测报告失败: {e}")


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

