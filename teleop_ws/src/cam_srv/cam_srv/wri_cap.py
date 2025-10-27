#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import asyncio
import signal
from typing import Any, Callable
import threading
import time



class Wri_Cam_Cap(Node):
    def __init__(
        self,
    ):
        super().__init__("WristCameraCapture")
        self.get_logger().info("Wrist camera node started")
        self.cam_pub_ = self.create_publisher(Image, "wrist_camera_Image", 10)
        self.bridge_ = CvBridge()
        # self.get_logger().info(f"{cv2.__file__}")
        self.target_fps = 10
        self.timer_period = 1.0 / self.target_fps
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)
        
        self.current_frame = None
        self.frame_lock = threading.Lock()
        self.running = True  

        self.cap = self.opencv_init()
        if not self.cap.isOpened():
            rclpy.shutdown()
            return

        self.read_thread = threading.Thread(target=self.read_frame, daemon=True)
        self.display_thread = threading.Thread(target=self.display, daemon=True)
        self.read_thread.start()
        self.display_thread.start()


    def opencv_init(self):
        cap = cv2.VideoCapture("/dev/video0",cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error("Error: Could not open video.")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        return cap

    def read_frame(self):
        while rclpy.ok() and self.running:
            try:
                ret,frame = self.cap.read()
                if ret is None:
                    continue
                with self.frame_lock:
                    self.current_frame = frame
            except Exception as e:
                self.get_logger().error(f"Error: {e}")

    def display(self):
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800, 600)
        while rclpy.ok() and self.running:            
            try:
                frame_to_display = None
                with self.frame_lock:
                    frame_to_display = self.current_frame
                if frame_to_display is not None:
                    cv2.imshow("frame", frame_to_display)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        self.running = False
                        break
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
        cv2.destroyAllWindows()

    def timer_callback(self):
        try:
            frame_to_publish = None
            with self.frame_lock:
                frame_to_publish = self.current_frame.copy() if self.current_frame is not None else None
            if frame_to_publish is not None:
                msg_Header = Header()
                msg_Header.stamp = self.get_clock().now().to_msg()

                image_msg = self.bridge_.cv2_to_imgmsg(frame_to_publish, encoding="bgr8",header=msg_Header)
                self.cam_pub_.publish(image_msg)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down camera node...")
        self.running = False  
        time.sleep(1)
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    wri_cam_cap = Wri_Cam_Cap()

    def handler(signum, frame):
        wri_cam_cap.get_logger().info("Received SIGINT, shutting down...")
        wri_cam_cap.destroy_node()
        rclpy.shutdown()
    signal.signal(signal.SIGINT, handler)
    try:
        rclpy.spin(wri_cam_cap)
    except KeyboardInterrupt:
        pass 
    finally:
        pass