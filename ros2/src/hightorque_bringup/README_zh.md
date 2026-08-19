# hightorque_bringup

`hightorque_bringup` 用来把机器人运行时需要的核心节点和配置组织在一起。
常见用途有两个：

1. 根据机器人构型自动找到对应配置目录
2. 一次性启动 midware、IMU、OLED、电源、手柄、等基础节点

当前主要启动文件是 [pi_plus_rknn.launch.py](launch/pi_plus_rknn.launch.py)。

## 配置目录

bringup 会根据 `model_type/leg_type/leg/arm/claw/head/waist` 组合出一个 `robot_type`，例如：

```text
pi_plus-S-12L8A0G2H0W
```

对应配置目录位于：

```text
hightorque_bringup/config/robot_config/<robot_type>/
```

这个目录里通常至少有：

- `robot_param.yaml`：机器人硬件和设备层配置，给底层驱动和一些外设节点使用
- `pd.yaml`：关节映射、限位、方向、默认 `kp/kd` 等控制配置，主要给 midware 使用
- `custom_action.yaml`：某些构型下的自定义动作配置

## pd.yaml 的意义

`pd.yaml` 不是单纯的 PD 参数表，它更像是“关节控制配置表”。
当前 `hightorque_midware_node` 会通过 `robot_config_path` 自动找到这个文件，并在启动时读取其中的配置。

路径示例：

```text
hightorque_bringup/config/robot_config/pi_plus-S-12L8A0G2H0W/pd.yaml
```

它主要决定以下事情：

- `joint_names`：关节名称顺序，和系统里的关节语义对应
- `map_index`：关节和电机 ID 的映射关系
- `direction`：正负方向修正，决定同样的角度命令在硬件上往哪个方向转
- `lower/upper`：关节限位，midware 下发命令前会参考这些范围
- `urdf_offset`：URDF 零位和真实电机零位之间的偏移
- `kp/kd`：默认控制增益
- `supports_kinematics`：是否启用正逆运动学相关能力

### midware 如何使用 pd.yaml

当前 `hightorque_midware_node` 会用 `pd.yaml` 完成这些初始化：

- 建立关节名到电机 ID 的映射
- 读取默认 `kp/kd`，作为位置控制时的兜底参数
- 读取 `direction`、`urdf_offset`，把软件坐标和硬件坐标对齐
- 读取 `lower/upper`，作为命令限幅依据

可以把它理解成：

- `robot_param.yaml` 更偏“设备和硬件连接”
- `pd.yaml` 更偏“关节控制和运动语义”

## bringup 启动了哪些节点

以下说明基于当前 [pi_plus_rknn.launch.py](launch/pi_plus_rknn.launch.py)。

### 1. hightorque_midware_node

包：`hightorque_midware`
可执行文件：`hightorque_midware_node`

作用：

- 负责电机控制权管理
- 对外提供 `/request_control`、`/release_control`、`/control_command` 等接口
- 把上层控制命令转换成底层电机命令
- 发布 `/joint_states` 和 session 状态

这是整套系统里最核心的“电机控制中间层”。
上层控制器、测试脚本、动作程序，一般都不是直接打电机，而是先经过它。

### 2. yesense_imu_node

包：`yesense_imu_ros2`
可执行文件：`yesense_imu_node`

作用：

- 读取 IMU 设备数据
- 发布姿态、角速度、加速度等惯导信息
- 为平衡控制、状态估计、策略控制提供躯干姿态输入

如果没有这个节点，依赖 IMU 的姿态控制和状态估计通常都无法正常工作。

### 3. hightorque_oled_node

包：`hightorque_oled`
可执行文件：`hightorque_oled_node`

作用：

- 驱动机器人机身上的 OLED 小屏
- 显示设备状态、运行状态或提示信息
- 让现场调试时不用连终端也能看到基础状态

这个节点本质上是“本机状态显示器”。

### 4. power_node

包：`hightorque_power`
可执行文件：`power_node`

作用：

- 负责电源相关状态管理
- 读取或发布电池/供电信息
- 承担机器人上电、供电监控、低电压告警等相关功能入口

它的具体话题和细节取决于 `hightorque_power` 包本身，但从 bringup 角度看，它就是整机电源管理节点。

### 5. joy_linux_node

包：`joy_linux`
可执行文件：`joy_linux_node`

作用：

- 从 `/dev/input/js0` 读取手柄原始输入
- 发布原始手柄消息
- 当前 launch 里把默认 `joy` 话题重映射成 `joy_input`

它只负责“采集原始手柄输入”，不负责业务语义转换。

### 6. joy_mapper_node.py

包：`hightorque_midware`
可执行文件：`joy_mapper_node.py`

作用：

- 订阅 `joy_input`
- 把 Linux 原始手柄按键/摇杆值转换成项目内部统一格式
- 输出 `hightorque_joy`

可以把它理解成“手柄协议转换层”。

### 7. humanoid_driver

包：`humanoid_driver`
可执行文件：`humanoid_driver`

作用：

- 手柄滤波


## 启动流程可以怎么理解

当前 bringup 的职责可以概括成：

1. 根据机器人构型找到配置目录
2. 启动硬件相关基础节点
3. 启动电机控制中间层 `hightorque_midware_node`
4. 启动手柄输入链路
5. 预留策略控制器接入位置

所以它更像“整机基础运行环境”，而不是单独某一个算法节点。

## 常用参数

当前 launch 文件声明了这些常用参数：

- `robot_id`：多机器人部署时用来加 namespace
- `model_type`：模型类型，如 `pi`、`pi_plus`、`hi`
- `leg_type/leg/arm/claw/head/waist`：拼出 `robot_type`
- `default_timeout_ms`：midware 控制超时时间
- `policy_name`：策略名称，供 policy controller 使用

其中：

- `robot_id` 为空时，不加 namespace
- `robot_id` 不为空时，节点会进入 `robot_<robot_id>` 命名空间

## 例子

例如：

```bash
ros2 launch hightorque_bringup pi_plus_rknn.launch.py   model_type:=pi_plus   leg_type:=S   leg:=12   arm:=8   claw:=0   head:=2   waist:=0
```

会解析出：

```text
robot_type = pi_plus-S-12L8A0G2H0W
```

然后主要加载：

- `hightorque_bringup/config/robot_config/pi_plus-S-12L8A0G2H0W/robot_param.yaml`
- `hightorque_bringup/config/robot_config/pi_plus-S-12L8A0G2H0W/pd.yaml`

## 调试建议

如果 bringup 后机器人行为不对，可以按下面顺序排查：

1. 先确认是否加载了正确的 `robot_type`
2. 再检查 `pd.yaml` 里的 `map_index/direction/urdf_offset/kp/kd`
3. 检查 `midware` 是否正常发布 `/joint_states`
4. 检查 IMU、power、joy 是否都正常启动
5. 最后再看策略层或动作层是否有问题
