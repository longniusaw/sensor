
#!/usr/bin/env bash

set -e

# 工作空间路径
WS_DIR="$HOME/robot_sensorTry_ws"

echo "===================================="
echo "  ROS 2 工作空间清理与编译"
echo "===================================="

# 进入工作空间
cd "$WS_DIR"

# 加载 ROS 2 Jazzy 环境
source /opt/ros/jazzy/setup.bash

echo ""
echo "[1/3] 清理旧的编译产物..."

rm -rf build install log

echo "旧的 build、install、log 已清理。"

echo ""
echo "[2/3] 开始重新编译..."

colcon build --packages-up-to lidarsensor

echo ""
echo "[3/3] 加载工作空间环境..."

source "$WS_DIR/install/setup.bash"

echo ""
echo "===================================="
echo "  编译成功！"
echo "===================================="
echo "运行节点的命令："
echo "ros2 run lidarsensor scan_filter_node"
