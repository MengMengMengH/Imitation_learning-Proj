#! /usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import os
import argparse

def set_academic_style():
    """设置学术论文级图表样式"""
    plt.rcParams.update({
        "font.family": "serif",
        "font.size": 11,
        "axes.labelsize": 13,
        "axes.titlesize": 14,
        "legend.fontsize": 11,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
        "lines.linewidth": 1.5,
        "axes.grid": True,
        "grid.alpha": 0.4,
        "grid.linestyle": "--"
    })

def plot_kinematics(model_name,t_start ,t_end,data_dir="q_sequence"):
    # 1. 莫兰迪高级色板 (最多支持 7 轴)
    morandi_colors = [
        '#5D7A8C', # 灰蓝色 (Joint 1)
        '#C0A88B', # 沙驼色 (Joint 2)
        '#9A8F97', # 藕粉紫 (Joint 3)
        '#7C8A79', # 鼠尾草绿 (Joint 4)
        '#D1B1A5', # 脏玫瑰色 (Joint 5)
        '#8C9A9E', # 雾霾灰 (Joint 6)
        '#D5C7B4'  # 浅米色 (Joint 7，若是 UR5 则用不到)
    ]

    # 2. 加载数据
    try:
        t_sec = np.load(os.path.join(data_dir, f"{model_name}_timestamps_ns.npy"))
        t_sec = (t_sec - t_sec[0])/ 1e9  # 时间归零
        
        q_rad = np.load(os.path.join(data_dir, f"{model_name}_joint_angles.npy"))
        qdot_rad = np.load(os.path.join(data_dir, f"{model_name}_joint_velocities.npy"))
    except FileNotFoundError as e:
        print(f"❌ 找不到数据文件: {e}")
        return
    
    mask = (t_sec >= t_start) & (t_sec <= t_end)

    if not np.any(mask):
        print(f"❌ 错误：在 {t_start}s 到 {t_end}s 之间没有找到任何数据！数据总时长为 {t_sec[-1]:.1f}s")
        return
    
    t_filtered = t_sec[mask]
    q_filtered = q_rad[mask]
    qdot_filtered = qdot_rad[mask]

    t_filtered = t_filtered - t_filtered[0]

    # 转换为角度制 (Degrees)，对人类读者更友好
    q_deg = q_filtered * (180.0 / np.pi)
    qdot_deg = qdot_filtered * (180.0 / np.pi)
    num_joints = q_deg.shape[1]

    # 3. 创建纵向堆叠的画布
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    
    # --- 上半部分：关节角度 (q) ---
    for j in range(num_joints):
        color = morandi_colors[j % len(morandi_colors)]
        ax1.plot(t_filtered, q_deg[:, j], label=f'Joint {j+1}', color=color, alpha=0.9)
    
    ax1.set_ylabel('Joint Angle (deg)')
    ax1.set_title(f'Kinematic States during Teleoperation ({model_name})')
    # 移除顶部和右侧边框
    ax1.spines['top'].set_visible(False)
    ax1.spines['right'].set_visible(False)
    
    # --- 下半部分：关节速度 (q_dot) ---
    for j in range(num_joints):
        color = morandi_colors[j % len(morandi_colors)]
        ax2.plot(t_filtered, qdot_deg[:, j], label=f'Joint {j+1}', color=color, alpha=0.8)
    
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Joint Velocity (deg/s)')
    # 移除顶部和右侧边框
    ax2.spines['top'].set_visible(False)
    ax2.spines['right'].set_visible(False)

    # 4. 图例处理 (统一放在图表右侧，避免遮挡曲线)
    # 获取 handles 和 labels (两个子图的标签是一样的)
    handles, labels = ax1.get_legend_handles_labels()
    fig.legend(handles, labels, loc='center right', bbox_to_anchor=(0.98, 0.5), title="Joints")
    
    # 调整布局，给右侧的图例留出空间
    plt.tight_layout()
    fig.subplots_adjust(right=0.85) 

    # 5. 保存图表
    save_png = f"{model_name}_kinematics.png"
    save_pdf = f"{model_name}_kinematics.pdf"
    fig.savefig(save_png, dpi=1000)
    # fig.savefig(save_pdf)
    print(f"✅ 成功生成运动学状态图: {save_png} / {save_pdf}")
    
    plt.show() 

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="绘制关节角与关节速度时序图")
    parser.add_argument('-m', '--model_name', type=str, required=True, help="机械臂型号")
    parser.add_argument('-d', '--data_dir', type=str, default='q_sequence', help="数据文件夹路径")
    
    parser.add_argument('--start', type=float, default=0.0, help="起始时间 (秒)，默认 0.0")
    parser.add_argument('--end', type=float, default=9999.0, help="结束时间 (秒)，默认全长")
    args = parser.parse_args()
    set_academic_style()
    plot_kinematics(args.model_name,args.start, args.end, args.data_dir)