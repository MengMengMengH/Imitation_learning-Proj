#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import queue
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

        self.target_fps = 30
        
        self.q: queue.Queue[Any] = queue.Queue()
        self.running = True  

        self.cap = self.opencv_init()

        self.read_thread = threading.Thread(target=self.read_frame, daemon=True)
        self.display_thread = threading.Thread(target=self.display, daemon=True)
        self.read_thread.start()
        self.display_thread.start()


    def opencv_init(self):
        cap = cv2.VideoCapture("/dev/video0",cv2.CAP_FFMPEG)
        if not cap:
            self.get_logger().error("Error: Could not open video.")
            rclpy.shutdown()
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        return cap

    def read_frame(self):
        while rclpy.ok() and self.running:
            try:
                ret,self.frame = self.cap.read()
                if ret is None:
                    continue
                if not self.q.empty():
                    self.q.get_nowait()
                self.q.put(self.frame)
            except queue.Empty:
                pass
            except Exception as e:
                self.get_logger().error(f"Error: {e}")

    def display(self):
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800, 600)
        while rclpy.ok() and self.running:
            try:
                frame = self.q.get()
                cv2.imshow("frame", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    self.running = False
                    break
            except queue.Empty:
                pass
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
        cv2.destroyAllWindows()

    def timer_callback(self):
        try:
            pass
            if self.frame is not None:
                self.frame_ = self.q.get()
                image_msg = self.bridge_.cv2_to_imgmsg(self.frame_, encoding="bgr8")
                self.cam_pub_.publish(image_msg)
        except queue.Empty:
            self.get_logger().error("Queue is empty")
            pass
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
        wri_cam_cap.destroy_node()
        rclpy.shutdown()