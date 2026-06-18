#!/bin/bash
# Orbbec Gemini Pro 药品识别系统启动脚本
# 首次使用请先安装 udev 规则（需要 sudo）：
#   cd /home/nvidia/OrbbecSDK_v1.10.35/Script
#   chmod +x install_udev_rules.sh
#   sudo ./install_udev_rules.sh
#   sudo udevadm control --reload-rules && sudo udevadm trigger

cd "$(dirname "$0")"

export PYTHONPATH="/home/nvidia/pyorbbecsdk/install/lib:$PYTHONPATH"
export LD_LIBRARY_PATH="/home/nvidia/pyorbbecsdk/install/lib:$LD_LIBRARY_PATH"

exec python3 orbbec_detect.py "$@"
