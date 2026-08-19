#!/usr/bin/env bash
# ============================================================
# walk_kick_framework — 新机器人环境一键安装脚本
#
# 用法:
#   1) 先把整个 walk_kick_framework 拷到机器人上 (建议 ~/walk_kick_framework):
#        rsync -av --delete --exclude={'.git','devel','build','install','log','.colcon','__pycache__','*.pyc','bags'} \
#          /home/tk/桌面/combine_lekai_vision/walk_kick_framework/ nvidia@<新机器人IP>:~/walk_kick_framework/
#   2) 在机器人上运行 (需 sudo):
#        bash ~/walk_kick_framework/install_robot_env.sh
#
# 平台要求:
#   - Ubuntu 20.04 aarch64 (Jetson), 已装 JetPack (带 CUDA / TensorRT / OpenCV 4.5)
#   - ZED 相机 (dvision 用 ZED SDK 3.x 取图; 可后装, 见 "ZED SDK" 段)
#   - yesense IMU 驱动 (现有机器人复用 ~/sim2real 的 yesense_imu_node)
#
# 脚本会逐项检测依赖, 缺什么装什么, 可重复执行 (幂等)。
#
# 可配置环境变量 (运行前 export):
#   ROBOT_USER   部署用户 (默认 $USER, 建议 nvidia)
#   ZED_SDK_URL  ZED SDK 安装包(.run)下载地址; 不设则跳过安装只给提示
#   YESENSE_WS   yesense_imu_node 所在 workspace (默认 ~/sim2real)
#   SKIP_ROS2    =1 跳过 ROS2 Foxy 安装
#   SKIP_ROS1    =1 跳过 ROS1 Noetic 安装
#   SKIP_ZED     =1 跳过 ZED SDK 安装
#   SKIP_CONDA   =1 跳过 miniconda/instinct_venv 安装
#   SKIP_BUILD   =1 跳过 ROS2/ROS1 构建
#   NO_CUDA      =1 构建 dvision 时不加 ZJUDANCER_GPU=1 (无 GPU 平台)
# ============================================================
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_USER="${ROBOT_USER:-$USER}"
YESENSE_WS="${YESENSE_WS:-$HOME/sim2real}"
ZED_SDK_URL="${ZED_SDK_URL:-}"

say()  { printf '\n\033[1;32m[+] %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m[!] %s\033[0m\n' "$*"; }
err()  { printf '\033[1;31m[ERROR] %s\033[0m\n' "$*" >&2; }

sudo_() { if [ "$(id -u)" -eq 0 ]; then "$@"; else sudo "$@"; fi; }

have() { command -v "$1" >/dev/null 2>&1; }

# ============================================================
# 0. 前置检查
# ============================================================
say "前置检查: 平台 / 架构 / CUDA"
if [ "$(uname -m)" != "aarch64" ]; then
  warn "当前架构 $(uname -m), 本框架按 Jetson aarch64 设计 (Ubuntu 20.04)。请确认机器人型号。"
fi
if ! grep -q "20.04" /etc/os-release 2>/dev/null; then
  warn "当前系统不是 Ubuntu 20.04 (ROS1 Noetic + ROS2 Foxy 需要)。请确认。"
fi
if ! have nvcc && [ -z "${NO_CUDA:-}" ]; then
  warn "未检测到 nvcc (CUDA)。dvision 需要 CUDA/JetPack。若无 GPU 可设 NO_CUDA=1 重新运行。"
fi
if [ "$(id -u)" -eq 0 ]; then
  warn "当前是 root 用户, 建议用普通用户 (nvidia) 运行, 这样 HOME/串口权限才正确。"
fi

# ============================================================
# 1. 系统依赖 + ROS apt 源
# ============================================================
say "安装系统依赖 + 配置 ROS apt 源"
if ! dpkg -l | grep -q "ros-foxy-ros-base"; then
  # ROS2 Foxy 源 (也覆盖 ROS1 Noetic)
  sudo_ sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list'
  sudo_ sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list'
  sudo_ apt-get install -y curl gnupg lsb-release
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo_ apt-key add -
fi
sudo_ apt-get update

# 基础构建依赖 (ROS1/ROS2 包分别在对应章节装)
sudo_ apt-get install -y \
  python3-pip python3-rosdep python3-catkin-pkg python3-rospkg python3-colcon-common-extensions \
  git cmake build-essential libboost-all-dev libeigen3-dev libgstreamer1.0-dev || \
  warn "部分基础 apt 包安装失败, 请检查上方错误"

# ============================================================
# 2. ROS2 Foxy
# ============================================================
if [ "${SKIP_ROS2:-0}" = "1" ]; then
  say "跳过 ROS2 Foxy 安装 (SKIP_ROS2=1)"
elif dpkg -l | grep -q "ros-foxy-ros-base"; then
  say "ROS2 Foxy 已安装, 跳过"
else
  say "安装 ROS2 Foxy"
  sudo_ apt-get install -y \
    ros-foxy-ros-base ros-foxy-rmw-cyclonedds-cpp ros-foxy-joy-linux \
    ros-foxy-launch ros-foxy-launch-ros ros-foxy-rosidl-default-generators \
    ros-foxy-rosidl-default-runtime ros-foxy-rclcpp ros-foxy-rclpy \
    ros-foxy-pluginlib ros-foxy-tf2-ros ros-foxy-geometry-msgs \
    ros-foxy-sensor-msgs ros-foxy-std-msgs ros-foxy-common-interfaces \
    ros-foxy-yaml-cpp-vendor ros-foxy-ros2bag ros-foxy-rosbag2 \
    libboost-serialization-dev python3-colcon-common-extensions python3-pip
  say "安装 ros1_bridge (IMU 桥)"
  sudo_ apt-get install -y ros-foxy-ros1-bridge
fi

# ============================================================
# 3. ROS1 Noetic
# ============================================================
if [ "${SKIP_ROS1:-0}" = "1" ]; then
  say "跳过 ROS1 Noetic 安装 (SKIP_ROS1=1)"
elif [ -f /opt/ros/noetic/setup.bash ]; then
  say "ROS1 Noetic 已安装, 跳过"
else
  say "安装 ROS1 Noetic"
  sudo_ apt-get install -y \
    ros-noetic-ros-base ros-noetic-common-msgs ros-noetic-image-transport \
    ros-noetic-tf2 ros-noetic-dynamic-reconfigure ros-noetic-rosbag
fi

# ============================================================
# 4. miniconda + instinct_venv (python3.8, ROS2 Foxy 需要)
# ============================================================
if [ "${SKIP_CONDA:-0}" = "1" ]; then
  say "跳过 conda/instinct_venv 安装 (SKIP_CONDA=1)"
else
  if [ ! -d "$HOME/miniconda3" ]; then
    say "安装 miniconda (aarch64) -> ~/miniconda3"
    curl -L -o /tmp/miniconda.sh https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh
    bash /tmp/miniconda.sh -b -p "$HOME/miniconda3"
  else
    say "miniconda 已存在, 跳过"
  fi
  # shellcheck disable=SC1091
  source "$HOME/miniconda3/etc/profile.d/conda.sh"
  if ! conda env list | awk '{print $1}' | grep -qx instinct_venv; then
    say "创建 instinct_venv (python3.8)"
    conda create -n instinct_venv python=3.8 -y
  fi
  say "安装 instinct_venv pip 依赖"
  conda activate instinct_venv
  pip install numpy numpy-quaternion pyyaml transformations prettytable empy==3.3.2 onnxruntime
  if [ ! -d "$HOME/ros2_numpy" ]; then
    git clone https://github.com/box-sort/ros2_numpy "$HOME/ros2_numpy"
  fi
  pip install -e "$HOME/ros2_numpy"
  conda deactivate
fi

# ============================================================
# 5. ZED SDK (dvision 取图, find_package(ZED 3 REQUIRED))
# ============================================================
if [ "${SKIP_ZED:-0}" = "1" ]; then
  say "跳过 ZED SDK 安装 (SKIP_ZED=1)"
elif [ -d /usr/local/zed ] && ls /usr/local/zed/lib/libzed.so* >/dev/null 2>&1; then
  ver="$(cat /usr/local/zed/ZED_SDK_VERSION 2>/dev/null || echo '?')"
  say "ZED SDK 已安装 (${ver}), 跳过"
elif [ -n "$ZED_SDK_URL" ]; then
  say "下载并安装 ZED SDK from $ZED_SDK_URL"
  curl -L -o /tmp/zed_sdk.run "$ZED_SDK_URL"
  chmod +x /tmp/zed_sdk.run
  # ZED 安装器静默参数可配置 (不同版本可能是 '-- silent' 或 '--silent')
  ZED_SDK_ARGS="${ZED_SDK_ARGS:--- silent}"
  sudo_ /tmp/zed_sdk.run $ZED_SDK_ARGS || {
    err "ZED SDK 安装失败 (参数: $ZED_SDK_ARGS)。"
    err "  请手动执行: sudo /tmp/zed_sdk.run  (交互式安装), 装到 /usr/local/zed 后重跑本脚本。"
  }
  rm -f /tmp/zed_sdk.run
else
  warn "未安装 ZED SDK, 且未设置 ZED_SDK_URL, 跳过。"
  warn "  dvision 构建需要 ZED SDK 3.x (find_package(ZED 3 REQUIRED))。"
  warn "  请到 https://www.stereolabs.com/developers/release/ 下载对应 JetPack 的 ZED SDK 并安装到 /usr/local/zed,"
  warn "  然后设置 ZED_SDK_URL 重跑本脚本, 或单独重编 dvision。"
fi

# ============================================================
# 6. 构建 ROS2 工作区
# ============================================================
if [ "${SKIP_BUILD:-0}" = "1" ]; then
  say "跳过构建 (SKIP_BUILD=1)"
else
  say "构建 ROS2 (colcon, 跳过 yesense_imu_ros2/hightorque_oled)"
  cd "$PROJECT_ROOT/ros2"
  # shellcheck disable=SC1091
  source /opt/ros/foxy/setup.bash
  colcon build --symlink-install --packages-skip yesense_imu_ros2 hightorque_oled --parallel-workers 2
fi

# ============================================================
# 7. 构建 ROS1 RoboCup_Workspace
# ============================================================
if [ "${SKIP_BUILD:-0}" != "1" ]; then
  # dvision 的 CMakeLists 硬编码 DANCER_LIB_PATH=/home/nvidia/RoboCup_Workspace/...
  # 若该路径不存在, 建一个指向本框架 RoboCup_Workspace 的软链, 保证能编过。
  if [ "$ROBOT_USER" = "nvidia" ] && [ ! -e /home/nvidia/RoboCup_Workspace ]; then
    say "创建 /home/nvidia/RoboCup_Workspace -> $PROJECT_ROOT/RoboCup_Workspace (dvision 硬编码路径)"
    sudo_ ln -sfn "$PROJECT_ROOT/RoboCup_Workspace" /home/nvidia/RoboCup_Workspace
  fi

  say "构建 ROS1 lib (dmsgs/dprocess/...)"
  cd "$PROJECT_ROOT/RoboCup_Workspace/lib"
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  catkin_make -j2

  say "构建 ROS1 core (dconfig -> dvision -> 其余)"
  cd "$PROJECT_ROOT/RoboCup_Workspace/core"
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  source ../lib/devel/setup.bash
  catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES=dconfig -j2
  source devel/setup.bash
  if [ -d /usr/local/zed ] || [ "${NO_CUDA:-0}" = "1" ]; then
    if [ "${NO_CUDA:-0}" = "1" ]; then
      say "构建 dvision (无 GPU, NO_CUDA=1)"
      catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES=dvision --pkg dvision -j2 || true
    else
      say "构建 dvision (CUDA, ZJUDANCER_GPU=1)"
      ZJUDANCER_GPU=1 catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES=dvision --pkg dvision -j2 || {
        err "dvision 构建失败。常见原因: ZED SDK 未装 / OpenCV 版本不是 4.5.4。"
        err "  - 装 ZED SDK 后重跑本脚本 (或单独重编 dvision)。"
        err "  - dvision 需要 find_package(OpenCV 4.5.4 REQUIRED), Jetson JetPack 自带 4.5.4。"
      }
    fi
  else
    warn "ZED SDK 未安装, 跳过 dvision 构建 (其余 ROS1 包继续)。装好 ZED 后需重编 dvision。"
  fi
  catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES='dconfig;dlaunch;dbehavior;dplanner;dancer-io;livelybot_msg;livelybot_serial;serial' -j2
fi

# ============================================================
# 8. yesense IMU 驱动
# ============================================================
say "检查 yesense IMU 驱动 (start_imu_bridge.sh 用)"
yesense_found=""
if ls "$YESENSE_WS"/install/yesense_imu*/lib/yesense_imu/yesense_imu_node >/dev/null 2>&1; then
  yesense_found="$YESENSE_WS"
elif find /opt/ros "$HOME" -maxdepth 5 -type f -name "yesense_imu_node" 2>/dev/null | grep -q .; then
  yesense_found="$(find /opt/ros "$HOME" -maxdepth 5 -type f -name 'yesense_imu_node' 2>/dev/null | head -1 | xargs dirname)"
fi
if [ -n "$yesense_found" ]; then
  say "  ✓ 找到 yesense_imu_node: $yesense_found"
else
  warn "未找到 yesense_imu_node。start_imu_bridge.sh 默认从 \$YESENSE_WS=~/sim2real 取 yesense 驱动。"
  warn "  若新机器人没有 sim2real, 需另装 yesense IMU 驱动, 或用框架自带 ros2 版 yesense (缺厂商 SDK, 默认跳过构建)。"
  warn "  可设置 YESENSE_WS=<含 yesense_imu_node 的 workspace> 重新运行。"
fi

# ============================================================
# 9. 禁用 sim2real 自启 (旧栈会抢串口/端口)
# ============================================================
say "禁用 sim2real 自启 (避免与足球框架抢串口)"
if systemctl list-unit-files 2>/dev/null | grep -q sim2real_master; then
  sudo_ systemctl disable sim2real_master.service 2>/dev/null || warn "  sim2real_master.service 禁用失败(可能已禁用)"
fi
if [ -f "$HOME/Desktop/sim2real.sh.desktop" ]; then
  sudo_ rm -f "$HOME/Desktop/sim2real.sh.desktop"
  warn "  已删除 ~/Desktop/sim2real.sh.desktop (开机自启)"
fi

# ============================================================
# 10. 串口权限 (电机板 ttyACM0-6 / IMU ttyUSB0)
# ============================================================
say "配置串口权限 (dialout 组)"
sudo_ usermod -aG dialout "$ROBOT_USER"
sudo_ usermod -aG dialout "$USER"
say "  ✓ 已把 $ROBOT_USER 和 $USER 加入 dialout 组 (需重新登录生效)"

# ============================================================
# 11. 收尾提示
# ============================================================
say "安装完成!"
echo ""
echo "============================================================"
echo " 下一步:"
echo "  1. 重新登录 (或 source ~/.bashrc), 让 dialout 组生效"
echo "  2. 检查电机板串口: ls /dev/ttyACM*   (应有 ttyACM0-6)"
echo "  3. 检查 IMU 串口:   ls /dev/ttyUSB*  (应有 ttyUSB0, CP2102N)"
echo "  4. 确认 ZED SDK:    ls /usr/local/zed  (dvision 需要)"
echo "  5. 启动: ./start_all.sh   (IMU桥 + bringup + 归零 + soccer + 视觉)"
echo ""
echo "  已知坑:"
echo "  - ~/.bashrc 若自动激活 conda base(py3.13) 会破坏 rclpy,"
echo "    所有 ROS2 命令须在 instinct_venv(py3.8) 下跑 (start_*.sh 已处理)。"
echo "  - dvision 构建失败先查 ZED SDK 是否在 /usr/local/zed。"
echo "  - ROS1 视觉依赖 ZED 相机独占访问。"
echo "============================================================"
