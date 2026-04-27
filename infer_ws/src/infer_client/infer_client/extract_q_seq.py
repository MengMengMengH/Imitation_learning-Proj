from drake_model import arm_ik,Quatnumpy_to_Rotation,load_config
from rosbags.highlevel import AnyReader
from pathlib import Path
import argparse
import yaml
import os
import numpy as np

def main():
    # 1. 配置参数
    bag_path = '/home/hanmg/Imitation_learning-Proj/raw_data/ros_topic/arm_action_test'  # 替换为你的 mcap 文件夹路径
    package_name = 'arm_hand_description'

    parser = argparse.ArgumentParser(description="离线提取机器人的关节数据")
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
        _, sdf_path = load_config(config_path, model_name)
        print(f"📄 成功加载 SDF 路径: {sdf_path}")
        # 如果你这里也需要启动 mujoco，可以直接使用 mjcf_path
        # print(f"📄 成功加载 MJCF 路径: {mjcf_path}")
    except Exception as e:
        print(e)
        return  # 读取配置失败则直接退出程序
    
    ik_solver = arm_ik(sdf_path, arm_type=model_name)
    q_last = ik_solver._q
    num_joints = len(q_last)

    # 3. 准备数据容器
    joint_angles = []
    timestamps = []
    
    # 4. 离线极速读取 MCAP 数据
    with AnyReader([Path(bag_path)]) as reader:
        connections = [x for x in reader.connections if x.topic == '/quat_data']
        if not connections:
            print("❌ 错误：在数据包中未找到 /quat_data 话题！")
            return
            
        print(f"✅ 开始处理 {model_name}，共需计算 {num_joints} 个关节...")

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
                timestamps.append(timestamp)
                joint_angles.append(q_sol.copy())

                q_last = q_sol
            else:
                # 如果某帧无解，为了保证画图时时序对齐，填入 NaN
                print(f"⚠️ 警告: 时间戳 {timestamp} 处 IK 求解失败")
                # timestamps.append(timestamp)

    q_array = np.array(joint_angles)  # Shape: (N, DOF)
    print(q_array)
    
    t_array = np.array(timestamps).astype(np.float64) / 1e9  # 纳秒转秒
    # print(len(q_array),len(t_array))

    q_vel = np.zeros_like(q_array)

    for j in range(num_joints):
        # 针对每个关节计算梯度
        q_vel[:, j] = np.gradient(q_array[:, j], t_array)

    # 5. 数据落盘保存为 .npy
    output_dir = "q_sequence"
    os.makedirs(output_dir, exist_ok=True)

    prefix = os.path.join(output_dir, f"{model_name}")

    np.save(f"{prefix}_joint_angles.npy", q_array)      # 🌟 保存角度
    np.save(f"{prefix}_joint_velocities.npy", q_vel)    # 🌟 保存速度
    np.save(f"{prefix}_timestamps_ns.npy", np.array(timestamps))
    
    print(f"🎉 处理完成！数据已存入 {output_dir}/")
    print(f"📊 角度形状: {q_array.shape}, 速度形状: {q_vel.shape}")

if __name__ == '__main__':
    main()


