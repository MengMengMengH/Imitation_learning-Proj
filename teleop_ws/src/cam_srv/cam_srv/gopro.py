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

from open_gopro import WiredGoPro, constants


class GoPro(Node):
    def __init__(
        self,
    ):
        super().__init__("WristCamera")
        self.get_logger().info("Wrist camera node started")
        self.cam_pub_ = self.create_publisher(Image, "wrist_camera_Image", 10)
        self.bridge_ = CvBridge()

        self.gopro = WiredGoPro()
        self.connected = False
        self.streaming = False
        self.target_fps = 30

        self.frame = None
        self.frame_lock = threading.Lock()

        self.q: queue.Queue[Any] = queue.Queue()

        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(self.connect_to_gopro())
        if self.connected:
            self.loop.run_until_complete(self.start_webcam())
        self.get_logger().info(f"start streaming:{self.streaming}")
        if self.streaming:
            self.timer_ = self.create_timer(0.01, self.timer_callback)
            self.get_logger().info("Webcam started, timer created")

            self.cap = self.opencv_init()
            if not self.cap:
                self.get_logger().error("Error: Could not open video.")
                self.loop.run_until_complete(self.stop_webcam())
                rclpy.shutdown()
            read_thread = threading.Thread(target=self.read_frame)
            read_thread.daemon = True

            display_thread = threading.Thread(target=self.display_thread)
            display_thread.daemon = True

            read_thread.start()
            display_thread.start()
        else:
            self.get_logger().error("Webcam not started,timer not created")
            rclpy.shutdown()

            # print("hello")

    async def connect_to_gopro(self,):
        try:
            await self.gopro.open()
            isready = await self.gopro.is_ready
            if isready:
                self.connected = True
                self.get_logger().info(
                    "Yay! I'm connected via USB, opened, and ready to send / get data now!"
                )
            else:
                self.get_logger().error(
                    "Boo! I'm not connected via USB, not opened, and not ready to send / get data now!"
                )
        except Exception as e:
            self.get_logger().error(f"Failed to connect to GoPro: {e}")

    async def start_webcam(self):
        try:
            await self.gopro.http_command.wired_usb_control(
                control=constants.Toggle.DISABLE
            )
            await self.gopro.http_command.set_shutter(shutter=constants.Toggle.DISABLE)

            status = await self.gopro.http_command.webcam_status()
            self.get_logger().info(f"Initial webcam status: {status.data}")
            info = await self.gopro.http_command.get_camera_info()
            self.get_logger().info(f"Camera info: {info}")

            if status.data.status not in {
                constants.WebcamStatus.OFF,
                constants.WebcamStatus.IDLE,
            }:
                self.get_logger().info("Webcam is not off or idle, stopping it")
                assert (await self.gopro.http_command.webcam_stop()).ok

            if (
                status := (await self.gopro.http_command.webcam_start()).data.error
            ) != constants.WebcamError.SUCCESS:
                self.get_logger().info(
                    f"{await self.gopro.http_command.webcam_start()}"
                )
                self.get_logger().error(f"Failed to start webcam: {status}")
                return -1
            self.streaming = True
        except Exception as e:
            self.get_logger().error(f"Failed to start webcam: {e}")

    async def stop_webcam(self):
        try:
            response = await self.gopro.http_command.webcam_stop()
            self.get_logger().info(f"Webcam stop response: {response}")
            self.streaming = False
            await self.gopro.http_command.webcam_exit()
        except Exception as e:
            self.get_logger().error(f"Failed to stop webcam: {e}")

    def opencv_init(self):
        stream_url = r"udp://172.28.197.51:8554"

        cap = cv2.cudacodec.createVideoReader(stream_url)
        # cap = cv2.VideoCapture(stream_url + "?overrun_nonfatal=1&fifo_size=50000000", cv2.CAP_FFMPEG)
        # cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # cap.set(cv2.CAP_PROP_N_THREADS, 4)
        return cap

    def read_frame(self):
        while self.streaming and rclpy.ok():
            try:
                # ret,self.frame = self.cap.read()
                ret, self.frame = self.cap.nextFrame()
                if ret is None:
                    continue
                if not self.q.empty():
                    self.q.get_nowait()
                self.q.put(self.frame)
            except queue.Empty:
                pass
            except Exception as e:
                self.get_logger().error(f"Error: {e}")

    def display_thread(self):
        while self.streaming and rclpy.ok():
            try:
                self.frame = self.q.get()
                frame_cpu = self.frame.download()
                cv2.imshow("frame", frame_cpu)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            except queue.Empty:
                pass
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
        # self.cap.release()
        cv2.destroyAllWindows()

    def timer_callback(self):
        if not self.streaming:
            return
        try:
            # with self.frame_lock:  # Acquire lock before reading frame
            pass
            # if self.frame is not None:
            #     self.frame_ = self.frame
            #     image_msg = self.bridge_.cv2_to_imgmsg(self.frame_, encoding="bgr8")
            #     self.cam_pub_.publish(image_msg)
            # self.get_logger().info(f"Frame: {type(q)}")
        except queue.Empty:
            self.get_logger().error("Queue is empty")
            pass
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    async def async_shutdown(self):
        if self.streaming:
            await self.stop_webcam()
        await self.gopro.close()

    def destroy_node(self):
        if self.connected:
            self.loop.run_until_complete(self.async_shutdown())
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    gopro = GoPro()

    def handler(signum, frame):
        gopro.get_logger().info("Received SIGINT, shutting down...")
        gopro.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, handler)
    try:
        rclpy.spin(gopro)
    except KeyboardInterrupt:
        pass  # 确保按 Ctrl+C 时不会抛异常
    finally:
        gopro.destroy_node()
        rclpy.shutdown()