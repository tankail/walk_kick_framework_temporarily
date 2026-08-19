# hightorque_kinematics_plugins

`hightorque_kinematics_plugins` 提供官方示例插件实现，供 `hightorque_midware` 通过 `pluginlib` 动态加载。

## 已有插件

- `hightorque/NoOpKinematicsPlugin`
  直接透传输入数据，不修改 `JointState` 和 `MotorControlCommand`。
- `hightorque/HeadInvertKinematicsPlugin`
  对 `head_yaw_joint` 和 `head_pitch_joint` 做取反处理。
  `forward()` 会把这两个关节的 `position/velocity/effort` 取反。
  `inverse()` 会把对应电机命令中的 `positions/velocities/torques` 取反。

插件导出列表见：

```xml
plugins.xml
```

## 配置方式

在 `bringup` 的 `joints.yaml` 中配置插件名：

```yaml
kinematics_plugin:
  name: "hightorque/NoOpKinematicsPlugin"
```

或者：

```yaml
kinematics_plugin:
  name: "hightorque/HeadInvertKinematicsPlugin"
```

如果不想使用插件：

```yaml
kinematics_plugin:
  name: ""
```

## 说明

- `midware` 只会在 `kinematics_plugin.name` 非空时尝试加载插件。
- 插件加载失败时，`midware` 会记录错误并回退到原始链路。
- 当前插件包依赖 `hightorque_kinematics_core`，运行前需要确保这两个包都已经编译并且 `source install/setup.bash`。
