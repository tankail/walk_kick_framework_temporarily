# walk_kick_framework — 精简足球框架

只保留 **走路找球(walk) / 踢球(kick) / 跌倒起立(fall-recovery)** 三个功能的精简足球框架,去掉裁判机(GameController/dnetwork)等无关功能。

**来源**: `piplus_soccer`(Route C),2026-08-06 精简。

## 手柄操作

| 组合键 | 作用 |
|---|---|
| **LT+RT+A** | 进入足球状态(走路找球/踢球/跌倒起立自动循环) |
| **LT+RT+B** | 退出足球状态 → 站立保持 + 停止策略运动(不释放电机,可再进入) |
| **LT+RT+START** | 紧急关机(释放电机 + 进程退出) |

> 原 LT/RT 单按紧急关机已移除,否则组合键不可用。

## 目录结构

```
walk_kick_framework/
├── start_terminal1_bringup.sh        # 终端1: bringup + 归零
├── start_terminal2_control_route_c.sh # 终端2: ROS2 运控 (soccer_control_node.py)
├── start_terminal3_ros1_decision.sh  # 终端3: ROS1 视觉 + 行为树
├── cleanup_soccer.sh                 # 清理所有进程
├── config/route_c_walk.yaml          # walk 策略选择 (h1w_locked_waist)
├── policy/piplus_walk_20260612_164526/  # walk onnx
├── RoboCup_Workspace/                # ROS1 (dvision/dbehavior/dplanner/dancer-io, 无 dnetwork)
└── ros2/                             # ROS2 hightorque + instinct_onboard_soccer
```

## 启动(3 终端)

```bash
# 终端1
./start_terminal1_bringup.sh
# 终端2 (等 cold start 完成后按 LT+RT+A)
./start_terminal2_control_route_c.sh
# 终端3
./start_terminal3_ros1_decision.sh
```

## 机器人部署 (192.168.99.151)

### 1. 装 ROS2 Foxy (需 sudo)
```bash
sudo apt update
sudo apt install -y ros-foxy-ros-base ros-foxy-rmw-cyclonedds-cpp \
  ros-foxy-joy-linux ros-foxy-launch ros-foxy-launch-ros \
  ros-foxy-rosidl-default-generators ros-foxy-rosidl-default-runtime \
  ros-foxy-rclcpp ros-foxy-rclpy ros-foxy-tf2-ros \
  ros-foxy-geometry-msgs ros-foxy-sensor-msgs ros-foxy-std-msgs \
  ros-foxy-common-interfaces ros-foxy-yaml-cpp-vendor \
  ros-foxy-ros2bag ros-foxy-rosbag2 \
  libboost-serialization-dev python3-colcon-common-extensions python3-pip
```

### 2. 建 instinct_venv (python3.8)
```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda create -n instinct_venv python=3.8 -y
conda activate instinct_venv
pip install numpy numpy-quaternion pyyaml transformations prettytable empy==3.3.2 onnxruntime
git clone https://github.com/box-sort/ros2_numpy ~/ros2_numpy || true
pip install -e ~/ros2_numpy
```

### 3. 构建 ROS2
```bash
cd ~/walk_kick_framework/ros2
source /opt/ros/foxy/setup.bash
# yesense_imu_ros2 / hightorque_oled 因缺厂商 SDK / serial_cmake 依赖而跳过, 不影响核心功能
colcon build --symlink-install --packages-skip yesense_imu_ros2 hightorque_oled --parallel-workers 2
```

### 3.5 IMU 方案 (ros1_bridge)

`yesense_imu_ros2` 依赖缺失的 yesense ROS2 SDK,改用**机器人已有的 ROS1 yesense 驱动 + ros1_bridge** 提供 `/imu`:

```bash
# (a) 装 ros1_bridge
sudo apt install -y ros-foxy-ros1-bridge

# (b) 启动 ROS1 roscore + yesense (ttyTHS0 是 IMU 串口, 0x5641 帧头已确认)
source /opt/ros/noetic/setup.bash
source ~/zx_workspace/sim2real_master-feature-master_and_slave/devel/setup.bash
roscore &
rosrun yesense_imu yesense_imu_node _yesense_port:=/dev/ttyTHS0 _yesense_baudrate:=460800 &
# 验证: rostopic echo -n1 /imu/data

# (c) 启动 ros1_bridge 把 ROS1 /imu/data 桥到 ROS2
source /opt/ros/foxy/setup.bash
source ~/walk_kick_framework/ros2/install/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics
# 或静态桥: ros2 run ros1_bridge parameter_bridge /imu/data@sensor_msgs/msg/Imu@[sensor_msgs/Imu
```

> 注意: 足球框架 HightorqueNode 订阅 `/imu`, 需确保桥出来的 topic 名是 `/imu` (可用 remap 或改 ros1_bridge 配置)。
> 若用动态桥, ROS1 `/imu/data` 会桥成 ROS2 `/imu/data`, 需在 soccer 节点或 launch 里 remap 为 `/imu`。

### 4. 构建 ROS1
```bash
cd ~/walk_kick_framework/RoboCup_Workspace/lib
source /opt/ros/noetic/setup.bash
catkin_make -j2

cd ~/walk_kick_framework/RoboCup_Workspace/core
source /opt/ros/noetic/setup.bash
source ../lib/devel/setup.bash
catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES=dconfig -j2
source devel/setup.bash
ZJUDANCER_GPU=1 catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES=dvision --pkg dvision -j2
catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES='dconfig;dlaunch;dbehavior;dplanner;dancer-io;livelybot_msg;livelybot_serial;serial' -j2
```

### 5. 同步代码
```bash
rsync -av --delete --exclude={'.git','devel','build','install','log','.colcon','__pycache__','*.pyc'} \
  /home/tk/桌面/combine_lekai_vision/walk_kick_framework/ nvidia@192.168.99.151:~/walk_kick_framework/
```

## 验证
- `ros2 topic echo /ros1_ball_position`(球位置, base_link 米)
- `ros2 topic hz /control_command`(~50Hz)
- 手柄: 冷启动 done 后 LT+RT+A 进入 / LT+RT+B 退出

## 关键技术点
- 去裁判机: `dbehavior` role 从 `GCStriker` 改 `Striker`(`role/normal.py`),不依赖 GCInfo。
- 手柄进入/退出: `soccer_control_node.py` 状态机 `cold_start →(LT+RT+A)→ soccer ⇄(LT+RT+B/LT+RT+A)⇄ hold`。
- hold 用 `_hold_current_h0w_pose()` 保持当前位姿,不释放电机控制。
- 紧急关机: `hightorque.py` 改为 LT+RT+START 三键组合。
- conda base(py3.13)会破坏 rclpy,所有 `ros2` 命令需在 `instinct_venv`(py3.8)下。
