#!/bin/bash

# ==========================================
# 自动化批量绘图参数配置区 (你可以随时修改这里)
# ==========================================
MODEL_NAME="rokae_7dof"
START_TIME=5.0
END_TIME=30.0
OFFSET=3.0

echo "=================================================="
echo "🚀 开始批量生成 ${MODEL_NAME} 的 1-7 轴运动学对比图"
echo "⏱️  时间截取: [${START_TIME}s - ${END_TIME}s], 偏移量: ${OFFSET}s"
echo "=================================================="

# 循环 1 到 7
for i in {1..7}
do
    echo "▶️  正在处理 Joint ${i} / 7 ..."
    
    # 执行你的 Python 绘图脚本
    python3 plt_comp.py -m ${MODEL_NAME} -j ${i} --start ${START_TIME} --end ${END_TIME} -o ${OFFSET}
    
    # 检查 Python 脚本是否成功运行 (返回值 0 表示成功)
    if [ $? -eq 0 ]; then
        echo "✅ Joint ${i} 处理完成！"
    else
        echo "❌ Joint ${i} 处理失败，脚本已中断！请检查报错信息。"
        exit 1  # 如果出错，直接停止整个脚本，防止生成一堆错图
    fi
    echo "--------------------------------------------------"
done
