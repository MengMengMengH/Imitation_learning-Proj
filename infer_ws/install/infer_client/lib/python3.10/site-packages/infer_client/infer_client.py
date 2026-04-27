import rclpy
from rclpy.node import Node
import zmq
import numpy as np
import time
import h5py
import cv2
import os
import sys
import csv
from datetime import datetime
import glob

from .drake_model import arm_ik,parse_arm_prediction

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        
        self.get_logger().info("🤖 [ROS 2] 正在初始化离线数据重放测试节点...")
        
        # ==========================================
        # 1. 声明并获取 ROS 2 传入的参数
        # ==========================================
        # 声明一个名为 'hdf5_path' 的参数，默认值为空字符串
        self.declare_parameter('dataset_dir', '')
        
        # 获取传入的参数值
        self.dataset_dir = self.get_parameter('dataset_dir').get_parameter_value().string_value
        
        # 检查是否正确传入了路径
        if not self.dataset_dir or not os.path.isdir(self.dataset_dir):
            self.get_logger().error("❌ 致命错误：未提供有效的数据集文件夹路径！")
            self.get_logger().info("👉 请使用: ros2 run <包名> <节点名> --ros-args -p dataset_dir:='你的包含多个hdf5的文件夹路径'")
            sys.exit(1)
        

        # ==========================================
        # 2. 建立 ZeroMQ 请求端 (REQ)
        # ==========================================
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.REQ)
        self.socket.connect("tcp://127.0.0.1:5555")
        self.get_logger().info("✅ [ROS 2] 已连接到本地 5555 端口的推理服务端")


        self.ik_solver = arm_ik('roake/urdf/roake_xMatePro3.sdf','roake_7dof')
        self.get_logger().info(f"🤖 机器人已成功加载至drake...")
        self.current_q = np.zeros(7)

        # ==========================================
        # 4. 初始化 CSV 数据记录器
        # ==========================================
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"eval_metrics_{timestamp}.csv"
        
        # 创建文件并写入表头 (Header)
        with open(self.csv_filename, mode='w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                "Test_Step", "Latency_ms", 
                "Arm_MSE_Total", "Arm_MSE_Step0", 
                "Hand_MSE_Total", "Hand_MSE_Step0",
                "Pos_Error_Step0_mm", "Ori_Error_Step0_deg",
                "Pos_Error_Step7_mm", "Ori_Error_Step7_deg"
            ])
        self.get_logger().info(f"📁 评估数据将实时追加保存至当前目录: {self.csv_filename}")

        self.h5_file = None
        self.current_episode_name = ""
        self.total_frames = 0

    def load_episode(self, hdf5_path):
        """🔴 新增：动态加载单个 HDF5 文件的逻辑"""
        # 如果上一个文件还开着，先安全关闭
        if self.h5_file is not None:
            self.h5_file.close()
            
        self.get_logger().info(f"📂 正在加载轨迹文件: {os.path.basename(hdf5_path)} ...")
        self.h5_file = h5py.File(hdf5_path, 'r')
        
        self.images = self.h5_file['images'][:] 
        self.arm_states = self.h5_file['observations']['arm_states'][:]
        self.hand_states = self.h5_file['observations']['hand_states'][:]
        self.force = self.h5_file['observations']['force'][:]
        self.actions_gt = self.h5_file['actions'][:]
        
        self.total_frames = self.images.shape[0]
        self.current_episode_name = os.path.basename(hdf5_path)
        self.get_logger().info(f"📊 加载完成，包含 {self.total_frames} 帧数据。")

    def calcu_cart_error(self,pred_18d,gt_18d,q_last):

        pred_rot_mat = parse_arm_prediction(pred_18d)
        print(pred_rot_mat)
        gt_rot_mat = parse_arm_prediction(gt_18d)

        q_pred = self.ik_solver.ori_inv(
            up_ori = pred_rot_mat[2],
            elbow_ori = pred_rot_mat[1],
            wrist_ori = pred_rot_mat[0],
            q_last = q_last
        )

        q_gt = self.ik_solver.ori_inv(
            up_ori=gt_rot_mat[2],
            elbow_ori=gt_rot_mat[1],
            wrist_ori=gt_rot_mat[0],
            q_last=q_last
        )

        if q_pred is None or q_gt is None:
            self.get_logger().warning("⚠️ IK 求解失败，跳过该帧误差计算。")
            print(f'{q_pred is None} , {q_gt is None}')
            return None, None
        
        self.ik_solver._q = q_pred
        _, _, _, T_pred = self.ik_solver.record_rot_data()
        pos_pred = T_pred.translation()
        rot_pred = T_pred.rotation().matrix()

        self.ik_solver._q = q_gt
        _, _, _, T_gt = self.ik_solver.record_rot_data()
        pos_gt = T_gt.translation()
        rot_gt = T_gt.rotation().matrix()

        pos_error_meters = np.linalg.norm(pos_pred - pos_gt)

        trace = np.clip(np.trace(np.dot(rot_pred.T, rot_gt)), -1.0, 3.0)
        ori_error_rad = np.arccos((trace - 1.0) / 2.0)

        return pos_error_meters, ori_error_rad



    def process_image(self, img):
        """将 HDF5 里的原始图像处理为 PyTorch 模型需要的格式"""
        # 1. Resize 到 320x240 (OpenCV 的尺寸格式是宽x高)
        resized = cv2.resize(img, (320, 240), interpolation=cv2.INTER_AREA)
        # 2. BGR 转 RGB
        rgb = resized[..., ::-1].copy()
        # 3. HWC 转 CHW (从 [240, 320, 3] 转成 [3, 240, 320])
        chw = np.transpose(rgb, (2, 0, 1))
        # 4. 转为 float32 并归一化到 [0, 1]
        return chw.astype(np.float32) / 255.0

    def get_random_observation(self):
        """随机抽取连续的两帧画面和状态，拼装成批次数据"""

        horizon = 16
        max_idx = self.total_frames - horizon - 1

        # 因为 n_obs_steps = 2，我们需要抽取 [idx] 和 [idx+1]，所以上限要 -2
        idx = np.random.randint(0, max_idx)
        self.get_logger().info(f"🎲 抽取时刻 t={idx} 与 t={idx+1} 的连续数据...")

        # 处理图像 -> 拼接成形状 (2, 3, 240, 320)
        img_t0 = self.process_image(self.images[idx])
        img_t1 = self.process_image(self.images[idx + 1])
        img_seq = np.stack([img_t0, img_t1], axis=0)

        # 处理状态 -> 拼接成形状 (2, 18)
        state_t0 = self.arm_states[idx].astype(np.float32)
        state_t1 = self.arm_states[idx + 1].astype(np.float32)
        state_seq = np.stack([state_t0, state_t1], axis=0)

        hand_t0 = self.hand_states[idx].astype(np.float32)
        hand_t1 = self.hand_states[idx + 1].astype(np.float32)
        hand_seq = np.stack([hand_t0, hand_t1], axis=0)

        force_t0 = self.force[idx].astype(np.float32)
        force_t1 = self.force[idx + 1].astype(np.float32)
        force_seq = np.stack([force_t0,force_t1],axis=0)

        obs_dict =  {
            'camera_1': img_seq,
            'robot_arm_state': state_seq,
            'robot_hand_state': hand_seq,
            'force_data': force_seq,
        }

        gt_action_seq = self.actions_gt[idx : idx + horizon].astype(np.float32)
        curr_arm_18d = self.arm_states[idx + 1].astype(np.float32)

        return obs_dict, gt_action_seq,curr_arm_18d



    def manual_control_loop(self):
        """使用按键触发的阻塞控制循环"""
        while rclpy.ok():
            # 阻塞等待用户在终端输入
            user_input = input("\n👉 按下【回车键 (Enter)】发送随机帧到模型 (输入 'q' 退出): ")
            if user_input.lower() == 'q':
                break
                
            self.step_counter += 1
            
            # 1. 获取包含时间维度的数据集观测
            obs_dict,gt_actions,curr_arm_18d = self.get_random_observation()
            
            # 2. 发送给 PyTorch 服务端
            start_time = time.time()
            self.socket.send_pyobj(obs_dict)
            
            # 3. 阻塞等待模型传回动作
            pred_actions = self.socket.recv_pyobj()
            latency = (time.time() - start_time) * 1000

            pred_arm = pred_actions[:, :18]
            pred_hand = pred_actions[:, 18:]
            
            gt_arm = gt_actions[:, :18]
            gt_hand = gt_actions[:, 18:]
            
            mse_arm_total = np.mean((pred_arm - gt_arm) ** 2)
            mse_hand_total = np.mean((pred_hand - gt_hand) ** 2)
            
            mse_arm_step0 = np.mean((pred_arm[0] - gt_arm[0]) ** 2)
            mse_hand_step0 = np.mean((pred_hand[0] - gt_hand[0]) ** 2)
            
            self.get_logger().info("=" * 50)
            self.get_logger().info(f"✅ [{self.step_counter}] 收到预测动作，总耗时: {latency:.1f} ms")
            self.get_logger().info(f"🦾 【手臂】(18维旋转矩阵) - 整体MSE: {mse_arm_total:.6f} | 第一步MSE: {mse_arm_step0:.6f}")
            self.get_logger().info(f"🖐️ 【手部】(6维大数值)    - 整体MSE: {mse_hand_total:.6f} | 第一步MSE: {mse_hand_step0:.6f}")
            self.get_logger().info("-" * 50)

            curr_rot_matrices = parse_arm_prediction(curr_arm_18d)
            current_q = self.ik_solver.ori_inv(
                up_ori=curr_rot_matrices[2],
                elbow_ori=curr_rot_matrices[1],
                wrist_ori=curr_rot_matrices[0],
                q_last=np.zeros(7)  # Roake 是 7自由度，必须是 np.zeros(7)
            )

            if current_q is None:
                self.get_logger().warning("⚠️ 当前帧初始 IK 求解失败，采用全 0 作为 Guess")
                current_q = np.zeros(7)

            pos_err_0, ori_err_0 = self.calcu_cart_error(pred_arm[0], gt_arm[0], q_last=current_q)
            pos_err_7, ori_err_7 = self.calcu_cart_error(pred_arm[7], gt_arm[7], q_last=current_q)


            if pos_err_0 is not None:
                self.get_logger().info(f"📍 【当前步 Step 0】 笛卡尔空间误差:")
                self.get_logger().info(f"   - 位移误差: {pos_err_0 * 1000:.2f} 毫米")
                self.get_logger().info(f"   - 姿态误差: {np.degrees(ori_err_0):.2f} 度")
                
                self.get_logger().info(f"📍 【未来步 Step 7】 笛卡尔空间误差:")
                if pos_err_7 is not None:
                    self.get_logger().info(f"   - 位移误差: {pos_err_7 * 1000:.2f} 毫米")
                    self.get_logger().info(f"   - 姿态误差: {np.degrees(ori_err_7):.2f} 度")
            self.get_logger().info("=" * 50)


            # =========================================================
            # 🔴 新增：将本次评估指标实时存入 CSV
            # =========================================================
            # 处理 IK 失败时的占位符 (存入 -1 表示无效数据)
            p0_mm = pos_err_0 * 1000 if pos_err_0 is not None else -1.0
            o0_deg = np.degrees(ori_err_0) if ori_err_0 is not None else -1.0
            
            p7_mm = pos_err_7 * 1000 if pos_err_7 is not None else -1.0
            o7_deg = np.degrees(ori_err_7) if ori_err_7 is not None else -1.0

            # 以追加模式 ('a') 打开文件，写完自动关闭，保证数据不丢失
            with open(self.csv_filename, mode='a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    self.step_counter, 
                    round(latency, 2),
                    mse_arm_total, 
                    mse_arm_step0,
                    mse_hand_total, 
                    mse_hand_step0,
                    p0_mm, 
                    o0_deg,
                    p7_mm, 
                    o7_deg
                ])

    def auto_evaluate_loop(self, evals_per_episode=1000):
        """全自动连续评估循环 (收到回复后立即发送下一次)"""
        self.get_logger().info(f"🚀 开始全自动离线评估，目标测试次数: {evals_per_episode} 次...")
        
        self.step_counter = 0
        # 记录整体开始时间
        eval_start_time = time.time()

        while rclpy.ok() and self.step_counter < evals_per_episode:
            self.step_counter += 1
            
            # 1. 随机获取观测
            obs_dict, gt_actions, curr_arm_18d = self.get_random_observation()
            
            # 2. 发送给 PyTorch 服务端
            start_time = time.time()
            self.socket.send_pyobj(obs_dict)
            
            # 3. 阻塞等待模型传回动作
            pred_actions = self.socket.recv_pyobj()
            latency = (time.time() - start_time) * 1000

            # --- 误差计算 (保持不变) ---
            pred_arm = pred_actions[:, :18]
            pred_hand = pred_actions[:, 18:]
            gt_arm = gt_actions[:, :18]
            gt_hand = gt_actions[:, 18:]
            
            mse_arm_total = np.mean((pred_arm - gt_arm) ** 2)
            mse_hand_total = np.mean((pred_hand - gt_hand) ** 2)
            mse_arm_step0 = np.mean((pred_arm[0] - gt_arm[0]) ** 2)
            mse_hand_step0 = np.mean((pred_hand[0] - gt_hand[0]) ** 2)
            
            # --- 笛卡尔误差计算 (保持不变) ---
            curr_rot_matrices = parse_arm_prediction(curr_arm_18d)
            current_q = self.ik_solver.ori_inv(
                up_ori=curr_rot_matrices[2],
                elbow_ori=curr_rot_matrices[1],
                wrist_ori=curr_rot_matrices[0],
                q_last=np.zeros(7)
            )
            
            if current_q is None:
                current_q = np.zeros(7)

            pos_err_0, ori_err_0 = self.calcu_cart_error(pred_arm[0], gt_arm[0], q_last=current_q)
            pos_err_7, ori_err_7 = self.calcu_cart_error(pred_arm[7], gt_arm[7], q_last=current_q)
            
            # --- 保存到 CSV (保持不变) ---
            p0_mm = pos_err_0 * 1000 if pos_err_0 is not None else -1.0
            o0_deg = np.degrees(ori_err_0) if ori_err_0 is not None else -1.0
            p7_mm = pos_err_7 * 1000 if pos_err_7 is not None else -1.0
            o7_deg = np.degrees(ori_err_7) if ori_err_7 is not None else -1.0

            if p0_mm > 50.0: # 如果误差大于 50 毫米 (5厘米)
                self.get_logger().error(f"🚨 发现极端异常误差: {p0_mm:.2f} mm!")
                self.get_logger().error(f"   -> 异常发生在数据集的 idx 附近 (请注意你代码里实际抽取的 idx)")
                
                self.get_logger().error(f"   -> 预测的 18D 向量: {pred_arm[0]}")
                self.get_logger().error(f"   -> 真实的 18D 向量: {gt_arm[0]}")

            with open(self.csv_filename, mode='a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    self.current_episode_name, self.step_counter, round(latency, 2),
                    mse_arm_total, mse_arm_step0, mse_hand_total, mse_hand_step0,
                    p0_mm, o0_deg, p7_mm, o7_deg
                ])

            # 为了避免终端输出太多看不清，每跑 10 次打印一次进度
            if self.step_counter % 10 == 0:
                progress = (self.step_counter / evals_per_episode) * 100
                self.get_logger().info(f"⏳ 进度: {self.step_counter}/{evals_per_episode} ({progress:.1f}%) | "
                                       f"最近一次延迟: {latency:.1f}ms | "
                                       f"Step0 位移误差: {p0_mm:.2f}mm")

        # 循环结束，打印总结
        total_time_min = (time.time() - eval_start_time) / 60
        self.get_logger().info("🎉" * 20)
        self.get_logger().info(f"✅ 评估完成！共测试 {self.step_counter} 次。")
        self.get_logger().info(f"⏱️ 总耗时: {total_time_min:.2f} 分钟。")
        self.get_logger().info(f"📁 数据已全部安全保存至: {self.csv_filename}")
        self.get_logger().info("🎉" * 20)

    def run_batch_evaluation(self, evals_per_episode=200):
            """🔴 新增：扫描文件夹，统筹管理整个批量测试流程"""
            # 使用 glob 找出目录下所有的 .hdf5 文件，并按名字排序保证顺序
            from pathlib import Path
            dataset_path = Path(self.dataset_dir)
            files_hdf5 = list(dataset_path.rglob('*.hdf5'))
            files_h5 = list(dataset_path.rglob('*.h5'))
            all_files = sorted([str(p) for p in (files_hdf5 + files_h5)])
            
            if not all_files:
                self.get_logger().error(f"❌ 文件夹 {self.dataset_dir} 中没有找到任何 .hdf5 文件！")
                return
                
            total_files = len(all_files)
            self.get_logger().info(f"🔍 扫描到 {total_files} 个轨迹文件，开始批量评估...")
            batch_start_time = time.time()

            # 核心外层循环：遍历每一个文件
            for i, file_path in enumerate(all_files):
                self.get_logger().info("=" * 60)
                self.get_logger().info(f"▶️ 正在处理第 {i+1}/{total_files} 个文件...")
                
                # 1. 加载当前文件
                rel_path = os.path.relpath(file_path, self.dataset_dir)
                self.get_logger().info(f"▶️ 正在处理第 {i+1}/{total_files} 个文件: {rel_path}")
                self.load_episode(file_path)
                
                # 2. 对当前文件执行多次抽样评估
                self.auto_evaluate_loop(evals_per_episode=evals_per_episode)

            # 扫尾工作
            total_time_min = (time.time() - batch_start_time) / 60
            self.get_logger().info("🎉" * 20)
            self.get_logger().info(f"✅ 批量评估全部完成！共测试了 {total_files} 个文件，总计 {total_files * evals_per_episode} 次推理。")
            self.get_logger().info(f"⏱️ 批量跑完总耗时: {total_time_min:.2f} 分钟。")
            self.get_logger().info(f"📁 汇总数据已存入: {self.csv_filename}")
            self.get_logger().info("🎉" * 20)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RobotControlNode()
        # 由用户的 input() 主导循环
        # node.manual_control_loop()
        node.run_batch_evaluation(200)
    except KeyboardInterrupt:
        pass # 静默处理 Ctrl+C
    except SystemExit:
        pass # 处理 sys.exit() 退出
    finally:
        # 确保安全关闭资源
        if 'node' in locals():
            if hasattr(node, 'h5_file') and node.h5_file:
                node.h5_file.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()