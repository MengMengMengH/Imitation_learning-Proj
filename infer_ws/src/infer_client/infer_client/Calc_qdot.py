from drake_model import arm_ik,Quatnumpy_to_Rotation,load_config
from rosbags.highlevel import AnyReader
from pathlib import Path
import argparse
import yaml
import os
import numpy as np

def main():
    # 1. 配置参数
    bag_path = '/home/hanmg/Imitation_learning-Proj/raw_data/ros_topic/arm_action_mpc'  # 替换为你的 mcap 文件夹路径

    parser = argparse.ArgumentParser(description="离线计算机器人的关节数据")
    # 添加必要的命令行参数
    parser.add_argument('-b', '--bag_path', type=str, default='/home/hanmg/Imitation_learning-Proj/raw_data/ros_topic/arm_action_mpc', help="[选填] mcap 数据包所在的文件夹路径 ")
    args = parser.parse_args()


    bag_path = args.bag_path

    # 3. 准备数据容器
    joint_angles = []
    timestamps = []
    
    # 4. 离线极速读取 MCAP 数据
    with AnyReader([Path(bag_path)]) as reader:
        connections = [x for x in reader.connections if x.topic == '/sent_joints']
        if not connections:
            print("❌ 错误：在数据包中未找到 /sent_joints 话题！")
            return
            

        # 遍历话题中的每一帧消息
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            # 将底层二进制数据反序列化为 Python 对象 (对应 std_msgs/msg/Float32MultiArray)
            msg = reader.deserialize(rawdata, connection.msgtype)
            joint_angle = np.array(msg.data)
            joint_angles.append(joint_angle)
            timestamps.append(timestamp)


    q_array = np.array(joint_angles)  # Shape: (N, DOF)
    num_joints = q_array.shape[1]
    
    t_array = np.array(timestamps).astype(np.float64) / 1e9  # 纳秒转秒
    # print(len(q_array),len(t_array))

    q_vel = np.zeros_like(q_array)

    for j in range(num_joints):
        # 针对每个关节计算梯度
        q_vel[:, j] = np.gradient(q_array[:, j], t_array)

    # 5. 数据落盘保存为 .npy
    output_dir = "q_sequence_filtered"
    os.makedirs(output_dir, exist_ok=True)

    prefix = os.path.join(output_dir,"mpc")

    np.save(f"{prefix}_joint_angles.npy", q_array)      # 🌟 保存角度
    np.save(f"{prefix}_joint_velocities.npy", q_vel)    # 🌟 保存速度
    np.save(f"{prefix}_timestamps_ns.npy", np.array(timestamps))
    
    print(f"🎉 处理完成！数据已存入 {output_dir}/")
    print(f"📊 角度形状: {q_array.shape}, 速度形状: {q_vel.shape}")

if __name__ == '__main__':
    main()


