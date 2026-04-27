import numpy as np
import matplotlib.pyplot as plt
import os
import csv
import argparse

morandi_colors = [
    "#8EB1B1",
    "#C39165",
    "#A56B6B",
]

def plot_efficiency(t_start,t_end,hz):
    suffix = f"_{t_start}s_{t_end}s" if (t_start > 0 or t_end < 9999) else ""
    csv_file = f"efficiency_metrics{suffix}.csv"
    file_exists = os.path.isfile(csv_file)

    with open(csv_file, mode='a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)

        # 如果是新文件，先写表头
        if not file_exists:
            header = ["Model", "Frame_Count", "Total_Time_s", "Avg_Time_ms", "P99_Time_ms", "Max_Time_ms"]
            writer.writerow(header)

        # 设置论文图表字体样式
        plt.rcParams.update({"font.family": "serif", "font.size": 12})
        data_dir = "Calc_times_ms"
        
        # 你的三个测试模型
        models = ["iiwa14_7dof", "UR5_6dof", "rokae_7dof"]
        
        all_times = []
        labels = []
        
        for model in models:
            times_file = os.path.join(data_dir, f"{model}_Solve_times_ms.npy")
            if not os.path.exists(times_file):
                print(f"⚠️ 找不到文件: {times_file}")
                continue

            times = np.load(times_file)
            t_sec = np.arange(len(times)) * (1.0 / hz)
                
            mask = (t_sec >= t_start) & (t_sec <= t_end)

            if not np.any(mask):
                print(f"⚠️ {model} 在指定时间段内无数据。总时长仅推算为 {t_sec[-1]:.1f}s")
                continue

            roi_times = times[mask]
            valid_times = roi_times[~np.isnan(roi_times)]

            if len(valid_times) == 0:
                print(f"⚠️ {model} 在该时间段内的有效数据为空。")
                continue

            # 绝对对齐：只有成功读取了数据，才同时 append 数据和标签
            all_times.append(valid_times)
            frame_count = len(valid_times)
            total_t_ms = np.sum(valid_times)
            total_t_s = total_t_ms / 1000.0  # 总耗时换算为秒更直观
            avg_t = np.mean(valid_times)
            max_t = np.max(valid_times)
            p99_t = np.percentile(valid_times, 99)
            row = f"{model:<15} | {frame_count:<8} | {total_t_s:<10.3f} | {avg_t:<12.2f} | {p99_t:<12.2f} | {max_t:<12.2f}"
            print(row)
            data_row = [model, frame_count, total_t_s, avg_t, p99_t, max_t]
            writer.writerow(data_row)

            print(f"✅ 数据已成功追加到 {csv_file}")
        
            # 截取型号前缀 (比如 UR5_6dof -> UR5)，让 X 轴文字更简洁好看
            short_name = model.split('_')[0] 
            labels.append(f"{short_name}\n(Avg: {avg_t:.2f}ms)")

    # 防御机制：如果没有数据则直接退出
    if len(all_times) == 0:
        print("❌ 没有有效数据用于绘图")
        return
        
    # 创建画布，稍微加宽一点以适配三个模型
    fig, ax = plt.subplots(figsize=(8, 6))
    
    # 绘制箱型图
    box = ax.boxplot(all_times, labels=labels, patch_artist=True, showfliers=True)
    
    for i, patch in enumerate(box['boxes']):
        # 使用取模运算 (%) 确保即使模型数量超过颜色数量也能循环使用
        color = morandi_colors[i % len(morandi_colors)]
        
        patch.set_facecolor(color)    # 设置填充颜色
        patch.set_edgecolor('#666666')  # 设置边框为深灰色，更柔和
        patch.set_alpha(0.9)          # 设置略高的透明度
        patch.set_linewidth(1.2)      # 略微加粗边框

        # 美化细节 (可选)
    ax.set_ylabel('Computation Time (ms)', fontsize=14)
    title_str = 'IK Solver Computational Efficiency'
    ax.set_title(title_str, fontsize=14)

    ax.grid(True, axis='y', linestyle='-', color='#EEEEEE', alpha=0.5) # 使用更淡的网格线
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    fig.tight_layout()

    # 保存高精度图片
    save_path = f"computational_efficiency_boxplot{suffix}.png"
    fig.savefig(save_path, dpi=1000)
    print(f"✅ 箱型图已成功生成: {save_path}")

    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="绘制截取时间段内的计算效率箱型图 (基于频率推算时间)")
    parser.add_argument('--start', type=float, default=0.0, help="起始时间 (秒)，默认 0.0")
    parser.add_argument('--end', type=float, default=48.0, help="结束时间 (秒)，默认全长")
    # 🌟 新增频率参数，默认 50Hz
    parser.add_argument('--hz', type=float, default=50.0, help="数据录制频率(Hz)，用于反推时间轴")
    args = parser.parse_args()
    
    plot_efficiency(args.start, args.end, args.hz)