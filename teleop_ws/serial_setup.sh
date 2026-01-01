#!/bin/bash
# Setup udev rules for robot serial ports

set -e  # 出错时退出

echo "=== 机器人串口设备设置 ==="
echo ""

# 检查是否为root用户
if [ "$EUID" -ne 0 ]; then 
    echo "需要sudo权限，请输入密码..."
    sudo "$0" "$@"
    exit $?
fi

echo "1. 创建udev规则文件..."
cat > /etc/udev/rules.d/99-robot-serial.rules << 'RULESEOF'
# Robot Serial Devices - Fixed Port Mapping
# Created: $(date)

# CP210x 
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
    SYMLINK+="teleop/glove", \
    MODE="0666", GROUP="dialout"

# FT232   
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", \
    SYMLINK+="teleop/force_sensor", \
    MODE="0666", GROUP="dialout"

# CH340 - Sensor Device
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
    SYMLINK+="teleop/inspire_hand", \
    MODE="0666", GROUP="dialout"
RULESEOF

echo "✅ 规则文件已创建: /etc/udev/rules.d/99-robot-serial.rules"

echo ""
echo "2. 重新加载udev规则..."
udevadm control --reload-rules
udevadm trigger

echo "✅ Udev规则已重新加载"

echo ""
echo "3. 检查dialout组..."
if ! groups $SUDO_USER | grep -q "dialout"; then
    echo "将用户 $SUDO_USER 添加到dialout组..."
    usermod -a -G dialout $SUDO_USER
    echo "⚠️  需要重新登录使组变更生效"
else
    echo "✅ 用户已在dialout组中"
fi

echo ""
echo "4. 当前udev规则列表:"
ls -l /etc/udev/rules.d/ | grep -E "99-robot|70-snap"

echo ""
echo "=== 设置完成 ==="
echo "请重新插拔USB设备，然后运行以下命令验证:"
echo "  ls -l /dev/teleop/"
