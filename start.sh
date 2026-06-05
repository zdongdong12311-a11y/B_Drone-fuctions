#!/bin/bash
# ==========================================================
# Highly Modular Autonomous Drone - 启动脚本
# 包含环境 Source、核心节点启动、一键退出和后台进程清理
# ==========================================================

echo "======================================================"
echo "    Highly Modular Autonomous Drone - 启动流程"
echo "======================================================"

# 1. 检查环境变量和路径
# 请根据实际工作空间路径调整
source /opt/ros/noetic/setup.bash

if [ -f ~/livox_ws/devel/setup.bash ]; then
    source ~/livox_ws/devel/setup.bash --extend
fi

if [ -f ~/fast_lio2_ws/devel/setup.bash ]; then
    source ~/fast_lio2_ws/devel/setup.bash --extend
fi

if [ -f ~/trans_ws/devel/setup.bash ]; then
    source ~/trans_ws/devel/setup.bash --extend
fi

if [ -f ~/ego_ws/devel/setup.bash ]; then
    source ~/ego_ws/devel/setup.bash --extend
fi

# 2. 清理退出函数
cleanup() {
    echo ""
    echo "======================================================"
    echo "  收到退出信号 (Ctrl+C)，开始清理后台进程..."
    echo "======================================================"
    # 安全关闭 roslaunch 进程
    killall -INT roslaunch 2>/dev/null
    sleep 3
    killall -9 roslaunch 2>/dev/null
    killall -9 rosmaster 2>/dev/null
    killall -9 rosout 2>/dev/null
    echo "清理完成！退出。"
    exit 0
}

# 捕获 SIGINT (Ctrl+C) 和 SIGTERM
trap cleanup SIGINT SIGTERM

# 3. 启动节点
echo "[1/2] 正在启动 LiDAR 驱动、FAST-LIO2 及 MAVROS 桥接..."
roslaunch lidar_to_mavros lidar_to_mavros.launch &
PID_LIDAR=$!
sleep 8 # 等待雷达与里程计稳定

echo "[2/2] 正在启动 EGO-Planner 轨迹规划器..."
roslaunch ego_planner single_run_in_exp.launch &
PID_EGO=$!
sleep 5

echo "======================================================"
echo " 所有核心节点已在后台启动！"
echo " "
echo " 请在新的终端运行自主导航脚本："
echo "   python3 navigation.py"
echo " "
echo " 保持当前终端打开，按 Ctrl+C 可一键结束所有进程。"
echo "======================================================"

# 挂起主线程，等待退出信号
wait $PID_LIDAR
wait $PID_EGO
