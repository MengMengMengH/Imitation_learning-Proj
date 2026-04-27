#! /usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches # 用于创建自定义图例
import os
import argparse

def set_academic_style():
    plt.rcParams.update({
        "font.family": "serif",
        "font.size": 12,
        "axes.labelsize": 14,
        "axes.titlesize": 15,
        "legend.fontsize": 11,
        "xtick.labelsize": 11,
        "ytick.labelsize": 11,
        "lines.linewidth": 2.0,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "grid.linestyle": "--"
    })

def find_nan_spans(t, data):
    """
    识别数据中 NaN 的连续区间
    返回: [(start_time, end_time), ...]
    """
    nan_indices = np.where(np.isnan(data))[0]
    if len(nan_indices) == 0:
        return []

    spans = []
    if len(nan_indices) > 0:
        start_idx = nan_indices[0]
        for i in range(1, len(nan_indices)):
            # 如果索引不连续，说明上一个 NaN 区间结束
            if nan_indices[i] != nan_indices[i-1] + 1:
                spans.append((t[start_idx], t[nan_indices[i-1]]))
                start_idx = nan_indices[i]
        spans.append((t[start_idx], t[nan_indices[-1]]))
    return spans

def plot_residuals(t_start, t_end, hz, data_dir="residual_results"):
    color_6dof = "#B85450" 
    color_7dof = "#6C8EBF" 
    color_failure = "#E0E0E0" # 淡淡的灰色
    
    model_6dof = "UR5_6dof"
    # model_7dof = "iiwa14_7dof"
    model_7dof = "rokae_7dof"
    
    # 1. 加载数据
    try:
        res_6dof = np.load(os.path.join(data_dir, f"{model_6dof}_shoulder_residuals.npy"))
        res_7dof = np.load(os.path.join(data_dir, f"{model_7dof}_shoulder_residuals.npy"))
    except FileNotFoundError as e:
        print(f"❌ 找不到文件: {e}")
        return

    min_len = min(len(res_6dof), len(res_7dof))
    t_sec = np.arange(min_len) * (1.0 / hz)
    res_6dof = res_6dof[:min_len]
    res_7dof = res_7dof[:min_len]

    # 2. 时间窗口截取
    mask = (t_sec >= t_start) & (t_sec <= t_end)
    t_filtered = t_sec[mask]
    res_6_filtered = res_6dof[mask]
    res_7_filtered = res_7dof[mask]

    # 3. 识别失败区间 (以 UR5 为主)
    failure_spans = find_nan_spans(t_filtered, res_6_filtered)

    # 4. 绘图
    fig, ax = plt.subplots(figsize=(11, 5))
    
    # 绘制数据线
    line7, = ax.plot(t_filtered, res_7_filtered * (180/np.pi), label='iiwa (7-DoF)', color=color_7dof, zorder=3)
    line6, = ax.plot(t_filtered, res_6_filtered * (180/np.pi), label='UR5 (6-DoF, Projection)', color=color_6dof, zorder=4)
    ax.fill_between(t_filtered, res_6_filtered * (180/np.pi), 0, color=color_6dof, alpha=0.1, zorder=2)

    # 绘制失败区域的垂直背景框
    for span in failure_spans:
        ax.axvspan(span[0], span[1], color=color_failure, alpha=0.5, lw=0, zorder=1)

    # 创建自定义图例项（Proxy Artist）来代表失败区域
    failure_patch = mpatches.Patch(color=color_failure, alpha=0.5, label='IK Failure / Out-of-Reach Zone')

    # 美化
    ax.axhline(0, color='black', linewidth=1, zorder=1)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Orientation Residual (deg)')
    ax.set_title('Submanifold Projection Residuals with Failure Zone Analysis')
    
    # 整合图例
    handles = [line7, line6, failure_patch]
    ax.legend(handles=handles, loc='upper left', framealpha=0.9)
    
    ax.set_xlim([t_filtered[0], t_filtered[-1]])
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    
    fig.tight_layout()

    save_path_png = f"residual_comparison.png"

    fig.savefig(save_path_png, dpi=1000)
    print(f"✅ 成功生成姿态残差对比图: {save_path_png}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--start', type=float, default=0.0)
    parser.add_argument('--end', type=float, default=45.0)
    parser.add_argument('--hz', type=float, default=50.0)
    args = parser.parse_args()
    set_academic_style()
    plot_residuals(args.start, args.end, args.hz)