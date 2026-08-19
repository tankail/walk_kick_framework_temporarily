#!/usr/bin/env bash
# walk_kick_framework 机器人部署脚本 (需要 sudo, 请用户在机器人上执行)
# 目标: 192.168.99.151 已同步 ~/walk_kick_framework
set -e

echo "======================================================"
echo " walk_kick_framework 部署 (需 sudo 密码)"
echo " 步骤: 1) 装 ROS2 Foxy  2) 建 instinct_venv  3) 构建 ROS2+ROS1"
echo "======================================================"

# ---------- 1. 安装 ROS2 Foxy (需 sudo) ----------
echo ""
echo "[1/4] 安装 ROS2 Foxy 基础包 ..."
sudo apt update
sudo apt install -y \
  ros-foxy-ros-base ros-foxy-rmw-cyclonedds-cpp ros-foxy-joy-linux \
  ros-foxy-launch ros-foxy-launch-ros ros-foxy-rosidl-default-generators \
  ros-foxy-rosidl-default-runtime ros-foxy-rclcpp ros-foxy-rclpy \
  ros-foxy-pluginlib ros-foxy-tf2-ros ros-foxy-geometry-msgs \
  ros-foxy-sensor-msgs ros-foxy-std-msgs ros-foxy-common-interfaces \
  ros-foxy-yaml-cpp-vendor ros-foxy-ros2bag ros-foxy-rosbag2 \
  libboost-serialization-dev python3-colcon-common-extensions python3-pip

# ---------- 2. 建 instinct_venv (python3.8, Foxy 编译绑定需要) ----------
echo ""
echo "[2/4] 创建 conda env instinct_venv (python3.8) ..."
source ~/miniconda3/etc/profile.d/conda.sh
if conda env list | awk '{print $1}' | grep -qx instinct_venv; then
  echo "  instinct_venv 已存在, 跳过创建"
else
  conda create -n instinct_venv python=3.8 -y
fi
conda activate instinct_venv
pip install numpy numpy-quaternion pyyaml transformations prettytable empy==3.3.2 onnxruntime
if [ ! -d "$HOME/ros2_numpy" ]; then
  git clone https://github.com/box-sort/ros2_numpy "$HOME/ros2_numpy"
fi
pip install -e "$HOME/ros2_numpy"

# ---------- 3. 构建 ROS2 ----------
echo ""
echo "[3/4] 构建 ROS2 工作区 (跳过 yesense_imu_ros2/hightorque_oled) ..."
cd "$HOME/walk_kick_framework/ros2"
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-skip yesense_imu_ros2 hightorque_oled --parallel-workers 2

# ---------- 3.5 装 ros1_bridge (IMU 桥) ----------
echo ""
echo "[3.5/4] 安装 ros1_bridge (IMU 桥接) ..."
sudo apt-get install -y ros-foxy-ros1-bridge

# ---------- 4. 构建 ROS1 (RoboCup_Workspace) ----------
echo ""
echo "[4/4] 构建 ROS1 RoboCup_Workspace ..."
cd "$HOME/walk_kick_framework/RoboCup_Workspace/lib"
source /opt/ros/noetic/setup.bash
catkin_make -j2

cd "$HOME/walk_kick_framework/RoboCup_Workspace/core"
source /opt/ros/noetic/setup.bash
source ../lib/devel/setup.bash
catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES=dconfig -j2
source devel/setup.bash
ZJUDANCER_GPU=1 catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES=dvision --pkg dvision -j2
catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES='dconfig;dlaunch;dbehavior;dplanner;dancer-io;livelybot_msg;livelybot_serial;serial' -j2

echo ""
echo "======================================================"
echo " 部署完成!"
echo " 启动方式 (3 终端):"
echo "   ./start_terminal1_bringup.sh   # bringup + 归零"
echo "   ./start_terminal2_control_route_c.sh  # 等冷启动后按 LT+RT+A"
echo "   ./start_terminal3_ros1_decision.sh"
echo "======================================================"
