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

def plot_joint_comparison(model_name,joint_idx,t_start ,t_end,offset,dir_ik="q_sequence",dir_mpc="q_sequence_filtered"):
    c_ang_ik = '#D89A9E'   # 浅粉红 (50Hz 原始角度)
    c_ang_mpc = '#B85450'  # 砖红色 (1000Hz MPC角度)
    
    c_vel_ik = '#A3B8CC'   # 浅灰蓝 (50Hz 原始速度)
    c_vel_mpc = '#5D7A8C'  # 深灰蓝 (1000Hz MPC速度)

    idx = joint_idx - 1 
    try:
        t_ik_raw = np.load(os.path.join(dir_ik, f"{model_name}_timestamps_ns.npy")).astype(np.float64) / 1e9
        q_ik = np.load(os.path.join(dir_ik, f"{model_name}_joint_angles.npy"))
        qd_ik = np.load(os.path.join(dir_ik, f"{model_name}_joint_velocities.npy"))
    except FileNotFoundError as e:
        print(f"❌ 找不到 IK 数据: {e}")
        return
    
    try:
        # 注意：这里尝试读取 mpc_ 前缀。如果你保存时前缀不一样，请修改这里
        t_mpc_raw = np.load(os.path.join(dir_mpc, "mpc_timestamps_ns.npy")).astype(np.float64) / 1e9
        q_mpc = np.load(os.path.join(dir_mpc, "mpc_joint_angles.npy"))
        qd_mpc = np.load(os.path.join(dir_mpc, "mpc_joint_velocities.npy"))
    except FileNotFoundError as e:
        print(f"❌ 找不到 MPC 数据: {e}")
        return
    
    if idx < 0 or idx >= q_ik.shape[1]:
        print(f"❌ 错误：请求的轴号 {joint_idx} 超出范围 (最大轴数为 {q_ik.shape[1]})")
        return
    
    t_ik = t_ik_raw - t_ik_raw[0]
    t_mpc = t_mpc_raw - t_mpc_raw[0]

    t_mpc = t_mpc + offset

    mask_ik = (t_ik >= t_start) & (t_ik <= t_end)
    mask_mpc = (t_mpc >= t_start) & (t_mpc <= t_end)
    if not np.any(mask_ik):
        print(f"❌ 错误：IK 数据在指定时间段 {t_start}s-{t_end}s 内无数据！")
        return
    if not np.any(mask_mpc):
        print(f"❌ 错误：MPC 数据在指定时间段 {t_start}s-{t_end}s 内无数据 (请检查 offset 是否过大)！")
        return
    
    t_ik_f = t_ik[mask_ik]
    q_ik_f = q_ik[mask_ik, idx] * (180.0 / np.pi)
    qd_ik_f = qd_ik[mask_ik, idx] * (180.0 / np.pi)

    t_mpc_f = t_mpc[mask_mpc]
    q_mpc_f = q_mpc[mask_mpc, idx] * (180.0 / np.pi)
    qd_mpc_f = qd_mpc[mask_mpc, idx] * (180.0 / np.pi)

    fig, ax1 = plt.subplots(figsize=(10, 5.5))
    
    # --- 左 Y 轴：绘制角度 (Angle) ---
    ax1.plot(t_ik_f, q_ik_f, label='IK Angle (50Hz)', color=c_ang_ik, linestyle='--', linewidth=2.0)
    ax1.plot(t_mpc_f, q_mpc_f, label='MPC Angle (1000Hz)', color=c_ang_mpc, linestyle='-', linewidth=2.5)
    
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel(f'Joint {joint_idx} Angle (deg)', color=c_ang_mpc, fontweight='bold')
    ax1.tick_params(axis='y', labelcolor=c_ang_mpc)
    ax1.grid(True, linestyle='--', alpha=0.3) # 仅开启左轴网格
    ax1.spines['top'].set_visible(False)

    # --- 右 Y 轴：绘制速度 (Velocity) ---
    ax2 = ax1.twinx()  # 🌟 创建共享 X 轴的第二 Y 轴
    ax2.plot(t_ik_f, qd_ik_f, label='IK Velocity (50Hz)', color=c_vel_ik, linestyle='--', linewidth=1.5, alpha=0.8)
    ax2.plot(t_mpc_f, qd_mpc_f, label='MPC Velocity (1000Hz)', color=c_vel_mpc, linestyle='-', linewidth=2.0, alpha=0.9)
    
    ax2.set_ylabel(f'Joint {joint_idx} Velocity (deg/s)', color=c_vel_mpc, fontweight='bold')
    ax2.tick_params(axis='y', labelcolor=c_vel_mpc)
    ax2.spines['top'].set_visible(False)

    # --- 图例合并处理 (底部水平排列，非常适合学术论文) ---
    lines_1, labels_1 = ax1.get_legend_handles_labels()
    lines_2, labels_2 = ax2.get_legend_handles_labels()
    
    # 将图例放在图表下方
    ax1.legend(lines_1 + lines_2, labels_1 + labels_2, 
               loc='upper center', bbox_to_anchor=(0.5, -0.15), 
               ncol=4, frameon=False)

    ax1.set_title(f'Tracking & Interpolation: Joint {joint_idx} ({model_name})', y=1.05)
    
    # 强制紧凑布局，防止底部图例被切掉
    plt.tight_layout()
    
    # 4. 保存图片
    suffix = f"_{t_start}s_{t_end}s_offset{offset}s" if (t_start > 0 or t_end < 9999 or offset != 0) else ""
    save_pdf = f"{model_name}_J{joint_idx}_Combined{suffix}.pdf"
    
    fig.savefig(save_pdf, format='pdf', bbox_inches='tight')
    # plt.show()
    print(f"✅ 成功生成单图双轴矢量图: {save_pdf}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="绘制指定关节的 IK 与 MPC 跟踪对比图")
    parser.add_argument('-m', '--model_name', type=str, required=True, help="机械臂型号")
    parser.add_argument('-j', '--joint', type=int, required=True, help="指定要查看的轴号 (例如: 1 代表 Joint 1)")
    
    parser.add_argument('--dir_ik', type=str, default='q_sequence', help="原始 IK 数据文件夹")
    parser.add_argument('--dir_mpc', type=str, default='q_sequence_filtered', help="MPC 滤波数据文件夹")
    
    parser.add_argument('--start', type=float, default=0.0, help="起始时间 (秒)，默认 0.0")
    parser.add_argument('--end', type=float, default=9999.0, help="结束时间 (秒)，默认全长")

    parser.add_argument('-o', '--offset', type=float, default=0.0, help="MPC时间偏移量(秒)。正数表示MPC数据往右平移，负数表示往左平移")
    args = parser.parse_args()
    
    set_academic_style()
    plot_joint_comparison(args.model_name, args.joint, args.start, args.end,args.offset, args.dir_ik, args.dir_mpc)