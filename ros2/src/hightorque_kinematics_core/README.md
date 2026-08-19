# hightorque_kinematics_core

`hightorque_kinematics_core` 提供 `midware` 使用的运动学插件基础接口。

## 接口

头文件位置：

```cpp
#include "hightorque_kinematics_core/hightorque_kinematics_plugin.hpp"
```

核心接口：

```cpp
struct KinematicsPluginConfig
{
  std::vector<std::string> jointNames;
  std::vector<int32_t> motorIds;
};

class KinematicsPlugin
{
public:
  virtual ~KinematicsPlugin() = default;

  virtual bool configure(const KinematicsPluginConfig & config) = 0;

  virtual bool forward(
    const sensor_msgs::msg::JointState & currentState,
    sensor_msgs::msg::JointState & publishedState) = 0;

  virtual bool inverse(
    const sensor_msgs::msg::JointState & currentState,
    const hightorque_msgs::msg::MotorControlCommand & command,
    hightorque_msgs::msg::MotorControlCommand & inversedCommand) = 0;

  virtual const KinematicsPluginConfig & config() const = 0;
};
```

## 语义

- `configure()`：由 `midware` 在加载插件后调用，传入当前机型的 `jointNames` 和 `motorIds`。
- `forward()`：用于状态发布链路。输入 `midware` 当前采集到的 `JointState`，输出最终要发布的 `JointState`。
- `inverse()`：用于指令下发链路。输入当前状态和上层传入的 `MotorControlCommand`，输出最终写入电机的 `MotorControlCommand`。
- 如果 `joints.yaml` 没有配置插件，`midware` 会直接旁路，不会调用这些接口。

## 配置来源

`midware` 从 `bringup` 的 `joints.yaml` 里读取插件配置：

```yaml
kinematics_plugin:
  name: ""
```

- `name` 为空字符串：不启用插件。
- `name` 非空：按 `pluginlib` 类名加载插件。

## 开发约定

- 插件类需要继承 `hightorque::KinematicsPlugin`。
- 插件类需要通过 `PLUGINLIB_EXPORT_CLASS(...)` 导出。
- 插件实现应尽量保持无副作用，不直接访问电机硬件。
