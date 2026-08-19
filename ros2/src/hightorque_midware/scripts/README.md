# 电机控制脚本使用说明

本目录包含多个Python脚本，用于测试和控制电机管理器。

## 脚本列表

### 1. move_to_zero.py - 电机归零脚本

**功能**: 申请全身电机控制权，读取当前位置，平滑过渡到零位

**使用方法**:
```bash
# 基本用法
python3 move_to_zero.py

# 或使用ros2 run
ros2 run hightorque_msgs move_to_zero.py
```

**运行流程**:
1. 申请全身电机控制权
2. 读取当前电机位置
3. 使用平滑S曲线插值，2秒内移动到零位
4. 保持零位2秒
5. 释放控制权（保持原状模式）

**特点**:
- 使用平滑插值（3t² - 2t³），避免突变
- 100Hz控制频率，确保平滑运动
- 实时打印进度信息
- 自动处理异常和中断信号

---

### 2. test_single_motor.py - 单电机测试脚本

**功能**: 申请单个电机控制权，进行正弦波运动测试

**使用方法**:
```bash
# 测试电机0（默认）
python3 test_single_motor.py

# 测试指定电机（例如电机5）
python3 test_single_motor.py 5

# 使用ros2 run
ros2 run hightorque_msgs test_single_motor.py 5
```

**运行流程**:
1. 申请指定电机控制权
2. 执行正弦波运动（振幅0.2rad，频率0.5Hz，持续5秒）
3. 返回初始位置
4. 释放控制权

**参数说明**:
- 命令行参数1: 电机ID（默认为0）

**特点**:
- 正弦波运动测试，验证电机响应
- 平滑返回初始位置
- 实时显示位置信息

---

### 3. test_all_motors.py - 全身电机测试脚本

**功能**: 自动检测所有电机并依次执行正弦波测试

**使用方法**:
```bash
# 基本用法
python3 test_all_motors.py

# 使用ros2 run
ros2 run hightorque_midware test_all_motors.py
```

**运行流程**:
1. 等待接收电机状态反馈
2. 查询所有可用电机
3. 从电机ID 0开始，依次对每个电机：
   - 申请单个电机控制权
   - 执行正弦波运动测试（5秒）
   - 返回初始位置
   - 释放控制权
4. 完成所有电机测试

**特点**:
- 自动检测电机数量，无需手动指定
- 串行测试，避免多电机同时运动
- 每个电机测试完成后自动返回初始位置
- 支持中断（Ctrl+C）安全退出
- 实时显示测试进度

---

### 4. monitor_motors.py - 电机状态监控脚本

**功能**: 实时监控所有电机的状态（位置、速度、力矩）

**使用方法**:
```bash
# 持续监控模式（默认，2Hz更新）
python3 monitor_motors.py

# 单次查询模式
python3 monitor_motors.py once

# 仅显示电机信息
python3 monitor_motors.py info

# 使用ros2 run
ros2 run hightorque_midware monitor_motors.py
```

**模式说明**:
- `continuous`: 持续监控，实时更新（默认）
- `once`: 单次查询，显示当前状态后退出
- `info`: 仅显示电机列表和状态（IDLE/OCCUPIED/ERROR）

**显示信息**:
- 活跃控制会话（UUID、节点名、控制的电机数）
- 电机ID、名称
- 当前位置（rad）
- 当前速度（rad/s）
- 当前力矩（N·m）
- 电机状态（IDLE/OCCUPIED/ERROR）
- 统计信息（范围、均值、标准差）

**特点**:
- 无需申请控制权，只读监控
- 每秒自动更新电机占用状态
- 实时显示活跃会话信息
- 清晰的表格显示
- 实时统计信息
- 支持多种显示模式

**监控界面示例**:

![电机状态监控 - 无活跃会话](../docs/images/monitor_1.png)

![电机状态监控 - 有活跃会话](../docs/images/monitor_2.png)


### 5. test_joints_individually.py 全身电机方向检测脚本

功能: 实现控制电机移动0.1弧度，观测是否正确移动，用于检验电机顺序和电机方向

使用方法:
```bash
ros2 run hightorque_midware test_joints_individually.py
```

如果需要修改测试的关节顺序以及要测试的关节，可修改test_joints_individually.py内的" self.target_joints  "

---

## 安装和配置

### 1. 添加可执行权限

```bash
cd /${path}/to/hightorque_midware/scripts
chmod +x *.py
```

### 2. 更新CMakeLists.txt

在`hightorque_midware/CMakeLists.txt`中添加：

```cmake
# 安装Python脚本
install(PROGRAMS
    scripts/move_to_zero.py
    scripts/test_single_motor.py
    scripts/monitor_motors.py
    DESTINATION lib/${PROJECT_NAME}
)
```

### 3. 重新编译

```bash
cd /${path}/
colcon build --packages-select hightorque_midware
source install/setup.bash
```

---

## 使用前提

1. **启动电机管理器节点**:
```bash
ros2 launch hightorque_bringup motor_only.launch.py
```

2. **确认服务可用**:
```bash
ros2 service list
```

应该看到：
- `/get_available_motors`
- `/request_control`
- `/release_control`

3. **确认话题可用**:
```bash
ros2 topic list
```

应该看到：
- `/control_command`
- `/joint_states`

---

## 常见问题

### Q1: 脚本无法找到模块

**错误**: `ModuleNotFoundError: No module named 'hightorque_msgs'`

**解决**:
```bash
source /${path}/install/setup.bash
```

### Q2: 申请控制权失败

**错误**: `Motor X is occupied by node: xxx`

**原因**: 电机已被其他节点占用

**解决**:
1. 检查是否有其他脚本正在运行
2. 等待其他节点释放控制权
3. 或者重启电机管理器节点

### Q3: 超时未接收到电机状态

**错误**: `超时：未接收到电机状态反馈`

**原因**: 电机管理器未正常运行或话题未发布

**解决**:
1. 确认电机管理器节点正在运行
2. 检查话题: `ros2 topic echo /joint_states`
3. 检查硬件连接

---

- [电机管理器README](../README.md)
- [ROS2 Python教程](https://docs.ros.org/en/foxy/Tutorials.html)
- [sensor_msgs/JointState](https://docs.ros2.org/foxy/api/sensor_msgs/msg/JointState.html)
