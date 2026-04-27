from drake_model import arm_ik,Quatnumpy_to_Rotation,load_config
from rosbags.highlevel import AnyReader
from pathlib import Path
import argparse
import os
import numpy as np


def main():
    # 1. 配置参数
    bag_path = '/home/hanmg/Imitation_learning-Proj/raw_data/ros_topic/arm_action_test'  # 替换为你的 mcap 文件夹路径
    package_name = 'arm_hand_description'

    parser = argparse.ArgumentParser(description="离线提取机器人的子流形投影残差数据")
    # 添加必要的命令行参数
    parser.add_argument('-m', '--model_name', type=str, required=True, help="[必填] 机器人的型号名称，例如 'UR5', 'iiwa14' 或 'roake_7dof'")
    parser.add_argument('-b', '--bag_path', type=str, default='/home/hanmg/Imitation_learning-Proj/raw_data/ros_topic/arm_action_test', help="[选填] mcap 数据包所在的文件夹路径 ")
    parser.add_argument('-c', '--config', type=str, default='models.yaml', help="[选填] yaml 配置文件的路径")
    args = parser.parse_args()

    model_name = args.model_name
    bag_path = args.bag_path
    config_path = args.config
    print(f"🚀 开始离线解析 ROS 2 数据包: {bag_path}")
    print(f"🤖 目标验证模型: {model_name}")
    # 2. 初始化 IK 求解器
    try:
        mjcf_path, sdf_path = load_config(config_path, model_name)
        print(f"📄 成功加载 SDF 路径: {sdf_path}")
        # 如果你这里也需要启动 mujoco，可以直接使用 mjcf_path
        # print(f"📄 成功加载 MJCF 路径: {mjcf_path}")
    except Exception as e:
        print(e)
        return  # 读取配置失败则直接退出程序
    
    ik_solver = arm_ik(sdf_path, arm_type=model_name)
    q_last = ik_solver._q
    
    # 3. 准备数据容器
    shoulder_residuals = []
    wrist_residuals = []
    timestamps = []
    
    # 4. 离线极速读取 MCAP 数据
    with AnyReader([Path(bag_path)]) as reader:
        connections = [x for x in reader.connections if x.topic == '/quat_data']
        if not connections:
            print("❌ 错误：在数据包中未找到 /quat_data 话题！")
            return
            
        print("✅ 找到话题，开始逐帧计算 IK 与投影残差...")
        count = 0
        # 遍历话题中的每一帧消息
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            # 将底层二进制数据反序列化为 Python 对象 (对应 std_msgs/msg/Float32MultiArray)
            msg = reader.deserialize(rawdata, connection.msgtype)
            
            # --- 以下是你原本 quat_callback 中的解析逻辑 ---
            quats = np.array(msg.data)
            rots = Quatnumpy_to_Rotation(quats)
            
            # 提取原始的人类期望姿态 (基于你的转换逻辑)
            # 注意：Drake 的 RotationMatrix 对象调用 .matrix() 返回 np.ndarray
            des_wrist_ori = rots[0]
            des_elbow_ori = rots[1]
            des_up_ori = rots[2]
            
            # --- 求解 IK ---
            q_sol = ik_solver.ori_inv(
                up_ori=des_up_ori.matrix(), 
                elbow_ori=des_elbow_ori.matrix(), 
                wrist_ori=des_wrist_ori.matrix(), 
                q_last=q_last
            )
            
            # --- 计算残差并记录 ---
            if q_sol is not None:
                # 调用我们之前写好的专门计算残差的函数
                # 传入未被投影污染的 des_up_ori 和 des_wrist_ori (作为 Drake RotationMatrix 对象传入)
                res_shoulder, res_wrist = ik_solver.calc_and_get_residuals(
                    q_sol, des_up_ori, des_wrist_ori
                )
                
                shoulder_residuals.append(res_shoulder)
                wrist_residuals.append(res_wrist)
                timestamps.append(timestamp)

                # 更新 q_last 为下一帧做准备
                q_last = q_sol
            else:
                # 如果某帧无解，为了保证画图时时序对齐，填入 NaN
                print(f"⚠️ 警告: 时间戳 {timestamp} 处 IK 求解失败")
                count += 1
                print(f'求解失败次数：{count}')
                shoulder_residuals.append(np.nan)
                wrist_residuals.append(np.nan)
                timestamps.append(timestamp)

    # 5. 数据落盘保存为 .npy
    output_dir = "residual_results"
    os.makedirs(output_dir, exist_ok=True)
    
    np.save(os.path.join(output_dir, f"{model_name}_shoulder_residuals.npy"), np.array(shoulder_residuals))
    np.save(os.path.join(output_dir, f"{model_name}_wrist_residuals.npy"), np.array(wrist_residuals))
    np.save(os.path.join(output_dir, "timestamps_ns.npy"), np.array(timestamps))
    
    print(f"🎉 处理完成！共提取 {len(shoulder_residuals)} 帧数据。")
    print(f"💾 结果已保存至 {output_dir}/ 目录下。")

if __name__ == '__main__':
    main()
