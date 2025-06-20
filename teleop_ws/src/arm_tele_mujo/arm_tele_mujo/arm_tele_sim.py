#! /usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Int32MultiArray


import mujoco
import mujoco.viewer
import numpy as np
import time
import threading
from pynput import keyboard

from .mjmodel import mj_armHandSys
from .arm_tele import ArmTele
from .drake_model import arm_ik,Quatnumpy_to_Rotation


class ArmTele_sim(ArmTele):
    def __init__(self,):
        super().__init__()

        self.quat_sub = self.create_subscription(
            Float32MultiArray,
            'quat_data', self.quat_callback, 10)
        self.model = self.robot.model
        self.data = self.robot.data

        self.exit_event = threading.Event() # 退出事件
        self.start_time = time.time()

        self._ik = arm_ik(f'{self.robot.sdf_model}',self.model_name)
        if  '_7dof' in self.model_name:
            self._q = np.array([0.2, 0.4, 0.6, 1.5, -0.3, -1.0,0.7])
        elif '_6dof' in self.model_name:
            self._q = np.array([0.2, 0.4, 0.6, 0.5, -0.3,0]) 

        self.inspire_q = np.zeros(12)

        self.viewer = None
        self.visual_thread = threading.Thread(target=self.visualize)
        self.visual_thread.start()
        print("visual_thread start")

        self.file_counter = 0
        self.save_dir = os.path.expanduser("~/teleop_ws/src/arm_tele_mujo/data")

        self.record_flag = False
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()
        
    def quat_callback(self, msg):
        now = int(time.time())
        self.quats = np.array(msg.data) # 50HZ
        rots = Quatnumpy_to_Rotation(self.quats)
        self._q = self._ik.ori_inv(up_ori=rots[2].matrix(),elbow_ori=rots[1].matrix(),wrist_ori=rots[0].matrix(),q_last=self._q)
        if self.viewer is not None:
            with self.viewer.lock():
                if self._q is not None:
                    self.data.ctrl = self._q 
                mujoco.mj_step(self.model, self.data)

                if self.record_flag:
                    real_Rs,real_Re,real_Rw,T = self._ik.record_rot_data()
                    des_Rs = rots[2].matrix()
                    des_Re = rots[1].matrix()
                    des_Rw = rots[0].matrix()
                    qpos = self.data.qpos
                    qvel = self.data.qvel
                    energy = self.data.energy
                    qacc = self.data.qacc

                    data = {
                        "real_Rs":real_Rs,
                        "real_Re":real_Re,
                        "real_Rw":real_Rw,
                        "des_Rs":des_Rs,
                        "des_Re":des_Re,
                        "des_Rw":des_Rw,
                        "qpos":qpos,
                        "qvel":qvel,
                        "qacc":qacc,
                        "energy":energy,
                        "real_T":T
                    }
                    npz_name = f'{self.model_name}_data_{now}_{self.file_counter}.npz'
                    full_path = os.path.join(self.save_dir, npz_name)
                    np.savez(full_path, **data)
                    self.file_counter += 1

    def visualize(self):
        try:
            mujoco.mj_resetData(self.model, self.data)
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.cam.lookat = np.array([0, 0, 0.5])
            self.viewer.cam.distance = 2.5
            self.viewer.cam.azimuth = 0
            # self.viewer.opt.frame = mujoco.mjtFrame.mjFRAME_BODY
            self.model.vis.scale.framewidth = 0.01
            self.model.vis.scale.framelength = 1

            while self.viewer.is_running() and not self.exit_event.is_set():
                # now = time.time()
                # with self.viewer.lock():
                #     mujoco.mj_step(self.model, self.data)

                self.viewer.sync()
                # rclpy.spin_once(self, timeout_sec=0.001)
        except Exception as e:
            self.get_logger().error(f"可视化线程错误：{e}")
        finally:
            if self.viewer is not None:
                self.viewer.close()  # 确保关闭 Viewer
            self.exit_event.set()
            self.get_logger().info("visual线程结束")

    def on_press(self,key):
        try:

            if key.char == 'c':
                self.record_flag = True
            elif key.char == 'v':
                self.record_flag = False

        except AttributeError:
            pass


def main(args=None):

    rclpy.init(args=args)
    arm_tele_sim = ArmTele_sim()

    try:
        rclpy.spin(arm_tele_sim)  # 运行ROS事件循环
    except KeyboardInterrupt:
        pass

    arm_tele_sim.destroy_node()
    rclpy.shutdown()