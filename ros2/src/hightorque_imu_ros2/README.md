# yesense_imu_ros2

高擎机器人 Yesense IMU 的 ROS2 功能包

## 功能

本功能包提供了 Yesense IMU 的 ROS2 接口，发布以下话题：

- `/yesense_imu/rpy` - 姿态角 (geometry_msgs/Vector3)
  - x: Roll (横滚角, 单位: 度)
  - y: Pitch (俯仰角, 单位: 度)
  - z: Yaw (偏航角, 单位: 度)

- `/yesense_imu/acc` - 加速度 (geometry_msgs/Vector3)
  - x, y, z: 线性加速度 (单位: m/s²)

- `/yesense_imu/gyro` - 角速度 (geometry_msgs/Vector3)
  - x, y, z: 角速度 (单位: °/s)

- `/yesense_imu/quat` - 四元数 (geometry_msgs/Quaternion)
  - w, x, y, z: 四元数表示的姿态

- `/yesense_imu/imu` - 标准 IMU 消息 (sensor_msgs/Imu)
  - 包含四元数、角速度（rad/s）和线性加速度的完整 IMU 数据

## 依赖

### 系统依赖
- ROS2 (Humble/Foxy/Galactic)
- yesense_imu 库 (需要先安装)

### ROS2 依赖
- rclcpp
- std_msgs
- geometry_msgs
- sensor_msgs

## 安装

### 1. 安装 yesense_imu 库

```bash
cd ~/GitRepository/hightorque_imu
mkdir build && cd build
cmake ..
make
sudo make install
```

### 2. 编译 ROS2 功能包

```bash
cd ~/ros2_ws
colcon build --packages-select yesense_imu_ros2
source install/setup.bash
```

## 使用方法

### 方法 1: 使用 launch 文件启动（推荐）

使用默认配置文件：
```bash
ros2 launch yesense_imu_ros2 yesense_imu.launch.py
```

使用自定义配置文件：
```bash
ros2 launch yesense_imu_ros2 yesense_imu.launch.py config_file:=/path/to/your/config.yaml
```

指定发布频率：
```bash
ros2 launch yesense_imu_ros2 yesense_imu.launch.py publish_rate:=200.0
```

同时指定配置文件和发布频率：
```bash
ros2 launch yesense_imu_ros2 yesense_imu.launch.py \
  config_file:=/path/to/your/config.yaml \
  publish_rate:=200.0
```

### 方法 2: 直接运行节点

使用默认配置：
```bash
ros2 run yesense_imu_ros2 yesense_imu_node
```

使用自定义配置文件：
```bash
ros2 run yesense_imu_ros2 yesense_imu_node --ros-args -p config_file:=/path/to/your/config.yaml
```

### 查看话题

```bash
# 查看所有话题
ros2 topic list

# 查看姿态角
ros2 topic echo /yesense_imu/rpy

# 查看加速度
ros2 topic echo /yesense_imu/acc

# 查看角速度
ros2 topic echo /yesense_imu/gyro

# 查看四元数
ros2 topic echo /yesense_imu/quat

# 查看标准 IMU 消息
ros2 topic echo /yesense_imu/imu
```

## 参数

- `publish_rate` (double, 默认: 100.0)
  - IMU 数据发布频率，单位 Hz

- `config_file` (string, 默认: "")
  - IMU 配置文件的完整路径
  - 如果为空，则使用默认路径查找配置文件
  - 使用 launch 文件时默认使用 ROS2 包中的配置文件

## 配置文件

### 配置文件位置

功能包自带配置文件：
```
~/ros2_ws/src/yesense_imu_ros2/config/imu_config.yaml
```

安装后的位置：
```
/opt/ros/<distro>/share/yesense_imu_ros2/config/imu_config.yaml
```

### 配置文件格式

```yaml
imu:
  pid: 0x5543           # USB产品ID
  vid: 0x5953           # USB供应商ID
  device: "/dev/ttyACM" # 串口设备前缀
  baud_rate: 460800     # 波特率
  lcm_enabled: false    # LCM使能
```

### 自定义配置

1. 复制默认配置文件：
   ```bash
   cp ~/ros2_ws/src/yesense_imu_ros2/config/imu_config.yaml ~/my_imu_config.yaml
   ```

2. 编辑配置文件，修改参数（如PID/VID、波特率等）

3. 使用自定义配置启动：
   ```bash
   ros2 launch yesense_imu_ros2 yesense_imu.launch.py \
     config_file:=~/my_imu_config.yaml
   ```

### 常见配置修改

**修改波特率**：
```yaml
baud_rate: 921600  # 改为更高的波特率
```

**修改设备前缀**（适用于不同的串口设备）：
```yaml
device: "/dev/ttyUSB"  # 改为 USB 转串口设备
```

**修改 PID/VID**（用于设备识别）：
```yaml
pid: 0xXXXX  # 你的设备的 Product ID
vid: 0xYYYY  # 你的设备的 Vendor ID
```

## 注意事项

1. **硬件连接**: 确保 IMU 设备已正确连接到电脑

2. **串口权限**: 确保当前用户有串口访问权限
   ```bash
   sudo usermod -a -G dialout $USER
   # 需要重新登录
   ```

3. **配置文件优先级**:
   - 如果指定了 `config_file` 参数，将使用该配置文件
   - 否则使用 ROS2 包中的默认配置文件
   - 如果都找不到，将尝试使用 yesense_imu 库的默认路径

4. **PID/VID**: 如果使用虚拟串口，需要在配置文件中正确设置 USB 设备的 PID 和 VID

## 故障排除

### 找不到 IMU 设备
- 检查设备是否已连接
- 检查配置文件中的 PID/VID 是否正确
- 确认用户有串口权限

### 找不到 yesense_imu 库
- 确保已安装 yesense_imu 库
- 如果安装到自定义路径，设置环境变量：
  ```bash
  export CMAKE_PREFIX_PATH=/your/install/path:$CMAKE_PREFIX_PATH
  ```

## 许可证

MIT
