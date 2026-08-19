# hightorque_midware

电机管理节点，负责分配电机控制权，并通过 ROS 2 服务/话题对外提供位控、力控和混合控制接口。

## 控制接口概览

默认启动后会提供以下接口：

- 服务：`/get_available_motors`
- 服务：`/request_control`
- 服务：`/release_control`
- 服务：`/reset_zero`
- 话题：`/control_command`
- 话题：`/joint_states`
- 话题：`/session_status`

如果你通过 namespace 启动节点，请把下面示例里的根路径替换成你的实际命名空间路径。

### 控制模式常量

- 位控：`hightorque_msgs/srv/RequestControl::POSITION_MODE = 0`
- 力控：`hightorque_msgs/srv/RequestControl::TORQUE_MODE = 1`
- 混合控制：`hightorque_msgs/srv/RequestControl::MIXED_MODE = 2`

### 释放/超时后的行为常量

- `KEEP_MODE = 0`：保持上一条指令
- `DAMPING_MODE = 1`：阻尼模式，推荐调试时使用
- `ZERO_TORQUE_MODE = 2`：零力矩

## 使用流程

无论是位控、力控还是混合控制，流程都一样：

1. 启动 `hightorque_midware_node`
2. 调用 `/request_control` 申请电机控制权，拿到 `uuid`
3. 持续向 `/control_command` 发布带 `uuid` 的控制消息
4. 调用 `/release_control` 主动释放控制权

## 注意事项

- 位控模式下，`positions` 是必填字段，长度必须和 `motor_ids` 一致；`torques` 会被忽略并按零处理。
- 力控模式下，不需要传 `positions`，只需要传 `torques`，长度必须和 `motor_ids` 一致；`position/kp/kd` 会被按零处理。
- 混合控制模式下，`positions`、`torques`、`kp`、`kd` 都必须和 `motor_ids` 等长。
- 混合控制里，位控电机请设置：`torques[i] = 0`，并填写该电机的 `positions[i]`、`kp[i]`、`kd[i]`。
- 混合控制里，力控电机请设置：`positions[i] = 0`、`kp[i] = 0`、`kd[i] = 0`，并填写该电机的 `torques[i]`。
- `velocities` 可以为空，也可以和 `motor_ids` 等长。
- 下面示例统一用电机 '0'、'1'、'2'、'3' 演示；控制更多电机时，把所有数组一起扩展到相同长度即可。

## 启动 midware

```bash
# 根据不同构型使用不同的启动文件
ros2 launch hightorque_bringup pi_plus_rknn.launch.py
```

也可以先确认接口是否在线：

```bash
ros2 service list | grep control
ros2 topic list | grep -E 'control_command|joint_states|session_status'
```

## 位控

### 终端命令模式

1. 申请位控权限：

```bash
ros2 service call /request_control \
  hightorque_msgs/srv/RequestControl \
  "{node_name: 'readme_position_cli', motor_ids: [0, 1, 2, 3], control_mode: 0, default_behavior: 1, timeout_ms: 5000, default_kp: [60.0, 60.0, 60.0, 60.0], default_kd: [1.0, 1.0, 1.0, 1.0]}"
```

记下返回结果里的 `uuid`，下面示例用 `YOUR_UUID` 代替。

2. 连续发布位控指令：

```bash
ros2 topic pub -r 100 /control_command hightorque_msgs/msg/MotorControlCommand \
  "{uuid: 'YOUR_UUID', motor_ids: [0, 1], positions: [0.2, -0.2, 0.2, -0.2], velocities: [], kp: [60.0, 60.0, 60.0, 60.0], kd: [1.0, 1.0, 1.0, 1.0], torques: []}"
```

3. 释放控制权：

```bash
ros2 service call /release_control \
  hightorque_msgs/srv/ReleaseControl \
  "{node_name: 'readme_position_cli', uuid: 'YOUR_UUID', release_mode: 1}"
```

### Python 方法

```python
import time

import rclpy
from rclpy.node import Node

from hightorque_msgs.msg import MotorControlCommand
from hightorque_msgs.srv import ReleaseControl, RequestControl


class MidwarePositionDemo(Node):
    def __init__(self):
        super().__init__('midware_position_demo')
        self.motor_ids = [0, 1, 2, 3]
        self.uuid = None

        self.request_client = self.create_client(RequestControl, '/request_control')
        self.release_client = self.create_client(ReleaseControl, '/release_control')
        self.cmd_pub = self.create_publisher(MotorControlCommand, '/control_command', 10)

    def wait_services(self):
        while not self.request_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting /request_control ...')
        while not self.release_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting /release_control ...')

    def request_control(self):
        req = RequestControl.Request()
        req.node_name = self.get_name()
        req.motor_ids = self.motor_ids
        req.control_mode = RequestControl.Request.POSITION_MODE
        req.default_behavior = RequestControl.Request.DAMPING_MODE
        req.timeout_ms = 5000
        req.default_kp = [60.0] * len(req.motor_ids)
        req.default_kd = [1.0] * len(req.motor_ids)

        future = self.request_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp is None or not resp.success:
            self.get_logger().error(f'request_control failed: {resp.message if resp else "no response"}')
            return False

        self.uuid = resp.uuid
        self.get_logger().info(f'control granted, uuid={self.uuid}')
        return True

    def publish_position(self, positions):
        msg = MotorControlCommand()
        msg.uuid = self.uuid
        msg.motor_ids = self.motor_ids
        msg.positions = positions
        msg.velocities = []
        msg.kp = [60.0, 60.0, 60.0, 60.0]
        msg.kd = [1.0, 1.0, 1.0, 1.0]
        msg.torques = []
        self.cmd_pub.publish(msg)

    def release_control(self):
        req = ReleaseControl.Request()
        req.node_name = self.get_name()
        req.uuid = self.uuid
        req.release_mode = ReleaseControl.Request.DAMPING_MODE

        future = self.release_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp is not None:
            self.get_logger().info(f'release_control: success={resp.success}, message={resp.message}')


def main():
    rclpy.init()
    node = MidwarePositionDemo()
    node.wait_services()

    try:
        if not node.request_control():
            return

        for _ in range(500):
            node.publish_position([0.2, -0.2, 0.2, -0.2])
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.1)
    finally:
        if node.uuid:
            node.release_control()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### C++ 方法

```cpp
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "hightorque_msgs/msg/motor_control_command.hpp"
#include "hightorque_msgs/srv/release_control.hpp"
#include "hightorque_msgs/srv/request_control.hpp"

class MidwarePositionDemo : public rclcpp::Node
{
public:
    MidwarePositionDemo()
        : Node("midware_position_demo"), motor_ids_{0, 1, 2, 3}
    {
        request_client_ = create_client<hightorque_msgs::srv::RequestControl>("/request_control");
        release_client_ = create_client<hightorque_msgs::srv::ReleaseControl>("/release_control");
        cmd_pub_ = create_publisher<hightorque_msgs::msg::MotorControlCommand>("/control_command", 10);
    }

    bool requestControl()
    {
        auto req = std::make_shared<hightorque_msgs::srv::RequestControl::Request>();
        req->node_name = get_name();
        req->motor_ids = motor_ids_;
        req->control_mode = hightorque_msgs::srv::RequestControl::Request::POSITION_MODE;
        req->default_behavior = hightorque_msgs::srv::RequestControl::Request::DAMPING_MODE;
        req->timeout_ms = 5000;
        req->default_kp = {60.0f, 60.0f, 60.0f, 60.0f};
        req->default_kd = {1.0f, 1.0f, 1.0f, 1.0f};

        auto future = request_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(shared_from_this(), future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "request_control failed");
            return false;
        }

        auto resp = future.get();
        if (!resp->success)
        {
            RCLCPP_ERROR(get_logger(), "request_control rejected: %s", resp->message.c_str());
            return false;
        }

        uuid_ = resp->uuid;
        RCLCPP_INFO(get_logger(), "control granted, uuid=%s", uuid_.c_str());
        return true;
    }

    void publishPosition(const std::vector<float>& positions)
    {
        hightorque_msgs::msg::MotorControlCommand msg;
        msg.uuid = uuid_;
        msg.motor_ids = motor_ids_;
        msg.positions = positions;
        msg.velocities = {};
        msg.kp = {60.0f, 60.0f, 60.0f, 60.0f};
        msg.kd = {1.0f, 1.0f, 1.0f, 1.0f};
        msg.torques = {};
        cmd_pub_->publish(msg);
    }

    void releaseControl()
    {
        auto req = std::make_shared<hightorque_msgs::srv::ReleaseControl::Request>();
        req->node_name = get_name();
        req->uuid = uuid_;
        req->release_mode = hightorque_msgs::srv::ReleaseControl::Request::DAMPING_MODE;
        release_client_->async_send_request(req);
    }

private:
    std::vector<int32_t> motor_ids_;
    std::string uuid_;
    rclcpp::Client<hightorque_msgs::srv::RequestControl>::SharedPtr request_client_;
    rclcpp::Client<hightorque_msgs::srv::ReleaseControl>::SharedPtr release_client_;
    rclcpp::Publisher<hightorque_msgs::msg::MotorControlCommand>::SharedPtr cmd_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MidwarePositionDemo>();

    if (node->requestControl())
    {
        for (int i = 0; i < 500; ++i)
        {
            node->publishPosition({0.0f, 0.0f, 0.0f, 0.0f});
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        node->releaseControl();
    }

    rclcpp::shutdown();
    return 0;
}
```

## 力控

这里的“力控”对应 `control_mode = 1`。底层同样统一下发 `MotorControlType::POS_VEL_TQE_KP_KD`，只是将 `kp/kd` 置零，主要通过 `torques` 控制输出。因此：

- 力控不需要传 `positions`
- `torques` 是主要的力矩输入
- `kp/kd` 不需要传；即使传了，midware 也会按零处理

### 终端命令模式

1. 申请力控权限：

```bash
ros2 service call /request_control \
  hightorque_msgs/srv/RequestControl \
  "{node_name: 'readme_torque_cli', motor_ids: [0, 1, 2, 3, 4, 5], control_mode: 1, default_behavior: 1, timeout_ms: 50000, default_kp: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], default_kd: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

2. 连续发布力控指令：

```bash
ros2 topic pub -r 100 /control_command hightorque_msgs/msg/MotorControlCommand \
  "{uuid: 'YOUR_UUID', motor_ids: [0, 1, 2, 3, 4, 5], positions: [], velocities: [], kp: [], kd: [], torques: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}"
```

3. 释放控制权：

```bash
ros2 service call /release_control \
  hightorque_msgs/srv/ReleaseControl \
  "{node_name: 'readme_torque_cli', uuid: 'YOUR_UUID', release_mode: 1}"
```

### Python 方法

```python
import time

import rclpy
from rclpy.node import Node

from hightorque_msgs.msg import MotorControlCommand
from hightorque_msgs.srv import ReleaseControl, RequestControl


class MidwareTorqueDemo(Node):
    def __init__(self):
        super().__init__('midware_torque_demo')
        self.motor_ids = [0, 1, 2, 3]
        self.uuid = None

        self.request_client = self.create_client(RequestControl, '/request_control')
        self.release_client = self.create_client(ReleaseControl, '/release_control')
        self.cmd_pub = self.create_publisher(MotorControlCommand, '/control_command', 10)

    def wait_services(self):
        while not self.request_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting /request_control ...')
        while not self.release_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting /release_control ...')

    def request_control(self):
        req = RequestControl.Request()
        req.node_name = self.get_name()
        req.motor_ids = self.motor_ids
        req.control_mode = RequestControl.Request.TORQUE_MODE
        req.default_behavior = RequestControl.Request.DAMPING_MODE
        req.timeout_ms = 5000
        req.default_kp = [0.0] * len(req.motor_ids)
        req.default_kd = [0.0] * len(req.motor_ids)

        future = self.request_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp is None or not resp.success:
            self.get_logger().error(f'request_control failed: {resp.message if resp else "no response"}')
            return False

        self.uuid = resp.uuid
        self.get_logger().info(f'control granted, uuid={self.uuid}')
        return True

    def publish_torque(self, torques):
        msg = MotorControlCommand()
        msg.uuid = self.uuid
        msg.motor_ids = self.motor_ids
        msg.positions = []
        msg.velocities = []
        msg.kp = []
        msg.kd = []
        msg.torques = torques
        self.cmd_pub.publish(msg)

    def release_control(self):
        req = ReleaseControl.Request()
        req.node_name = self.get_name()
        req.uuid = self.uuid
        req.release_mode = ReleaseControl.Request.DAMPING_MODE
        future = self.release_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)


def main():
    rclpy.init()
    node = MidwareTorqueDemo()
    node.wait_services()

    try:
        if not node.request_control():
            return

        for _ in range(500):
            node.publish_torque([0.15, -0.15, 0.15, -0.15])
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.1)
    finally:
        if node.uuid:
            node.release_control()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### C++ 方法

```cpp
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "hightorque_msgs/msg/motor_control_command.hpp"
#include "hightorque_msgs/srv/release_control.hpp"
#include "hightorque_msgs/srv/request_control.hpp"

class MidwareTorqueDemo : public rclcpp::Node
{
public:
    MidwareTorqueDemo()
        : Node("midware_torque_demo"), motor_ids_{0, 1}
    {
        request_client_ = create_client<hightorque_msgs::srv::RequestControl>("/request_control");
        release_client_ = create_client<hightorque_msgs::srv::ReleaseControl>("/release_control");
        cmd_pub_ = create_publisher<hightorque_msgs::msg::MotorControlCommand>("/control_command", 10);
    }

    bool requestControl()
    {
        auto req = std::make_shared<hightorque_msgs::srv::RequestControl::Request>();
        req->node_name = get_name();
        req->motor_ids = motor_ids_;
        req->control_mode = hightorque_msgs::srv::RequestControl::Request::TORQUE_MODE;
        req->default_behavior = hightorque_msgs::srv::RequestControl::Request::DAMPING_MODE;
        req->timeout_ms = 5000;
        req->default_kp = {0.0f, 0.0f};
        req->default_kd = {0.0f, 0.0f};

        auto future = request_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(shared_from_this(), future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "request_control failed");
            return false;
        }

        auto resp = future.get();
        if (!resp->success)
        {
            RCLCPP_ERROR(get_logger(), "request_control rejected: %s", resp->message.c_str());
            return false;
        }

        uuid_ = resp->uuid;
        RCLCPP_INFO(get_logger(), "control granted, uuid=%s", uuid_.c_str());
        return true;
    }

    void publishTorque(const std::vector<float>& torques)
    {
        hightorque_msgs::msg::MotorControlCommand msg;
        msg.uuid = uuid_;
        msg.motor_ids = motor_ids_;
        msg.positions = {};
        msg.velocities = {};
        msg.kp = {};
        msg.kd = {};
        msg.torques = torques;
        cmd_pub_->publish(msg);
    }

    void releaseControl()
    {
        auto req = std::make_shared<hightorque_msgs::srv::ReleaseControl::Request>();
        req->node_name = get_name();
        req->uuid = uuid_;
        req->release_mode = hightorque_msgs::srv::ReleaseControl::Request::DAMPING_MODE;
        release_client_->async_send_request(req);
    }

private:
    std::vector<int32_t> motor_ids_;
    std::string uuid_;
    rclcpp::Client<hightorque_msgs::srv::RequestControl>::SharedPtr request_client_;
    rclcpp::Client<hightorque_msgs::srv::ReleaseControl>::SharedPtr release_client_;
    rclcpp::Publisher<hightorque_msgs::msg::MotorControlCommand>::SharedPtr cmd_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MidwareTorqueDemo>();

    if (node->requestControl())
    {
        for (int i = 0; i < 500; ++i)
        {
            node->publishTorque({0.15f, -0.15f});
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        node->releaseControl();
    }

    rclcpp::shutdown();
    return 0;
}
```

## 混合控制

这里的“混合控制”对应 `control_mode = 2`。底层仍统一下发 `MotorControlType::POS_VEL_TQE_KP_KD`，但每个电机按你传入的数组分别决定是走位控还是力控：

- 位控电机：`torques[i] = 0`，同时设置 `positions[i]`、`kp[i]`、`kd[i]`
- 力控电机：`positions[i] = 0`、`kp[i] = 0`、`kd[i] = 0`，同时设置 `torques[i]`
- 混合模式下，`positions`、`torques`、`kp`、`kd` 都必须和 `motor_ids` 等长

### 终端命令模式

下面示例中：

- 电机 `0`、`1` 走力控，目标力矩分别为 `0.15`、`0.10`
- 电机 `2`、`3` 走位控，目标位置分别为 `0.2 rad`、`-0.2 rad`

```bash
ros2 service call /request_control \
  hightorque_msgs/srv/RequestControl \
  "{node_name: 'readme_mixed_cli', motor_ids: [0, 1, 2, 3], control_mode: 2, default_behavior: 1, timeout_ms: 5000, default_kp: [0.0, 0.0, 60.0, 60.0], default_kd: [0.0, 0.0, 1.0, 1.0]}"
```

```bash
ros2 topic pub -r 100 /control_command hightorque_msgs/msg/MotorControlCommand \
  "{uuid: 'YOUR_UUID', motor_ids: [0, 1, 2, 3], positions: [0.0, 0.0, 0.2, -0.2], velocities: [0.0, 0.0, 0.0, 0.0], kp: [0.0, 0.0, 60.0, 60.0], kd: [0.0, 0.0, 1.0, 1.0], torques: [0.15, 0.10, 0.0, 0.0]}"
```

```bash
ros2 service call /release_control \
  hightorque_msgs/srv/ReleaseControl \
  "{node_name: 'readme_mixed_cli', uuid: 'YOUR_UUID', release_mode: 1}"
```

### Python 方法

```python
import time

import rclpy
from rclpy.node import Node

from hightorque_msgs.msg import MotorControlCommand
from hightorque_msgs.srv import ReleaseControl, RequestControl


class MidwareMixedDemo(Node):
    def __init__(self):
        super().__init__('midware_mixed_demo')
        self.motor_ids = [0, 1, 2, 3]
        self.uuid = None

        self.request_client = self.create_client(RequestControl, '/request_control')
        self.release_client = self.create_client(ReleaseControl, '/release_control')
        self.cmd_pub = self.create_publisher(MotorControlCommand, '/control_command', 10)

    def wait_services(self):
        while not self.request_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting /request_control ...')
        while not self.release_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting /release_control ...')

    def request_control(self):
        req = RequestControl.Request()
        req.node_name = self.get_name()
        req.motor_ids = self.motor_ids
        req.control_mode = RequestControl.Request.MIXED_MODE
        req.default_behavior = RequestControl.Request.DAMPING_MODE
        req.timeout_ms = 5000
        req.default_kp = [0.0, 0.0, 60.0, 60.0]
        req.default_kd = [0.0, 0.0, 1.0, 1.0]

        future = self.request_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp is None or not resp.success:
            self.get_logger().error(f'request_control failed: {resp.message if resp else "no response"}')
            return False

        self.uuid = resp.uuid
        return True

    def publish_mixed(self):
        msg = MotorControlCommand()
        msg.uuid = self.uuid
        msg.motor_ids = self.motor_ids
        msg.positions = [0.0, 0.0, 0.2, -0.2]
        msg.velocities = [0.0, 0.0, 0.0, 0.0]
        msg.kp = [0.0, 0.0, 60.0, 60.0]
        msg.kd = [0.0, 0.0, 1.0, 1.0]
        msg.torques = [0.15, 0.10, 0.0, 0.0]
        self.cmd_pub.publish(msg)

    def release_control(self):
        req = ReleaseControl.Request()
        req.node_name = self.get_name()
        req.uuid = self.uuid
        req.release_mode = ReleaseControl.Request.DAMPING_MODE
        future = self.release_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)


def main():
    rclpy.init()
    node = MidwareMixedDemo()
    node.wait_services()

    try:
        if not node.request_control():
            return

        for _ in range(500):
            node.publish_mixed()
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.1)
    finally:
        if node.uuid:
            node.release_control()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### C++ 方法

```cpp
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "hightorque_msgs/msg/motor_control_command.hpp"
#include "hightorque_msgs/srv/release_control.hpp"
#include "hightorque_msgs/srv/request_control.hpp"

class MidwareMixedDemo : public rclcpp::Node
{
public:
    MidwareMixedDemo()
        : Node("midware_mixed_demo"), motor_ids_{0, 1, 2, 3}
    {
        request_client_ = create_client<hightorque_msgs::srv::RequestControl>("/request_control");
        release_client_ = create_client<hightorque_msgs::srv::ReleaseControl>("/release_control");
        cmd_pub_ = create_publisher<hightorque_msgs::msg::MotorControlCommand>("/control_command", 10);
    }

    bool requestControl()
    {
        auto req = std::make_shared<hightorque_msgs::srv::RequestControl::Request>();
        req->node_name = get_name();
        req->motor_ids = motor_ids_;
        req->control_mode = hightorque_msgs::srv::RequestControl::Request::MIXED_MODE;
        req->default_behavior = hightorque_msgs::srv::RequestControl::Request::DAMPING_MODE;
        req->timeout_ms = 5000;
        req->default_kp = {0.0f, 0.0f, 60.0f, 60.0f};
        req->default_kd = {0.0f, 0.0f, 1.0f, 1.0f};

        auto future = request_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(shared_from_this(), future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "request_control failed");
            return false;
        }

        auto resp = future.get();
        if (!resp->success)
        {
            RCLCPP_ERROR(get_logger(), "request_control rejected: %s", resp->message.c_str());
            return false;
        }

        uuid_ = resp->uuid;
        return true;
    }

    void publishMixed()
    {
        hightorque_msgs::msg::MotorControlCommand msg;
        msg.uuid = uuid_;
        msg.motor_ids = motor_ids_;
        msg.positions = {0.0f, 0.0f, 0.2f, -0.2f};
        msg.velocities = {0.0f, 0.0f, 0.0f, 0.0f};
        msg.kp = {0.0f, 0.0f, 60.0f, 60.0f};
        msg.kd = {0.0f, 0.0f, 1.0f, 1.0f};
        msg.torques = {0.15f, 0.10f, 0.0f, 0.0f};
        cmd_pub_->publish(msg);
    }

    void releaseControl()
    {
        auto req = std::make_shared<hightorque_msgs::srv::ReleaseControl::Request>();
        req->node_name = get_name();
        req->uuid = uuid_;
        req->release_mode = hightorque_msgs::srv::ReleaseControl::Request::DAMPING_MODE;
        release_client_->async_send_request(req);
    }

private:
    std::vector<int32_t> motor_ids_;
    std::string uuid_;
    rclcpp::Client<hightorque_msgs::srv::RequestControl>::SharedPtr request_client_;
    rclcpp::Client<hightorque_msgs::srv::ReleaseControl>::SharedPtr release_client_;
    rclcpp::Publisher<hightorque_msgs::msg::MotorControlCommand>::SharedPtr cmd_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MidwareMixedDemo>();

    if (node->requestControl())
    {
        for (int i = 0; i < 500; ++i)
        {
            node->publishMixed();
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        node->releaseControl();
    }

    rclcpp::shutdown();
    return 0;
}
```

## 零位校准

### 概述

`/reset_zero` 服务用于将全部电机的当前位置设为零位，即把电机当前的机械位置记录为 `position = 0`，并持久化到电机内部。

**前置条件：**
- midware 节点正常运行
- **没有任何活跃的控制会话**（所有节点必须先释放控制权）
- 电机处于可通信状态

**注意事项：**
- 校准过程中电机会被置为零力矩，请确保机器人处于0位支架上
- 服务调用为同步阻塞，完成后才返回结果
- 默认超时时间为 10000ms，可通过 `timeout_ms` 参数调整
- 判断校准完成的依据：所有电机上报的位置绝对值 ≤ 0.1 rad
- 无论成功或失败，都要重启电机的电源

**服务定义：**

```
# 请求
int32 timeout_ms    # 超时（毫秒），<=0 时使用默认值 10000ms

---
# 响应
bool success
uint8 status        # 0=FAILED  1=SUCCEEDED  2=TIMEOUT
string message
```

### 校准流程

1. 确认所有控制节点已释放控制权
2. 将机器人摆到目标零位姿态（手动辅助到位）
3. 调用 `/reset_zero` 服务
4. 等待服务返回，确认 `success = true`

### 终端命令

```bash
ros2 service call /reset_zero \
  hightorque_msgs/srv/ResetZero \
  "{timeout_ms: 10000}"
```

返回示例（成功）：
```
success: True
status: 1
message: Reset zero completed successfully
```

返回示例（超时，部分电机未响应）：
```
success: False
status: 2
message: Reset zero timed out, pending motors: 2, 5
```

返回示例（有活跃会话，拒绝执行）：
```
success: False
status: 0
message: Cannot reset zero while control sessions are active
```

### Python 方法

```python
import rclpy
from rclpy.node import Node
from hightorque_msgs.srv import ResetZero


class ResetZeroClient(Node):
    def __init__(self):
        super().__init__('reset_zero_client')
        self.client = self.create_client(ResetZero, '/reset_zero')

    def call(self, timeout_ms: int = 10000):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting /reset_zero ...')

        req = ResetZero.Request()
        req.timeout_ms = timeout_ms

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()

        if resp is None:
            self.get_logger().error('No response from /reset_zero')
            return False

        if resp.success:
            self.get_logger().info(f'Reset zero succeeded: {resp.message}')
        else:
            self.get_logger().warn(f'Reset zero failed (status={resp.status}): {resp.message}')

        return resp.success


def main():
    rclpy.init()
    node = ResetZeroClient()
    try:
        node.call(timeout_ms=10000)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### C++ 方法

```cpp
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "hightorque_msgs/srv/reset_zero.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("reset_zero_client");
    auto client = node->create_client<hightorque_msgs::srv::ResetZero>("/reset_zero");

    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(node->get_logger(), "waiting /reset_zero ...");
    }

    auto req = std::make_shared<hightorque_msgs::srv::ResetZero::Request>();
    req->timeout_ms = 10000;

    auto future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Service call failed");
        rclcpp::shutdown();
        return 1;
    }

    auto resp = future.get();
    if (resp->success)
    {
        RCLCPP_INFO(node->get_logger(), "Reset zero succeeded: %s", resp->message.c_str());
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Reset zero failed (status=%d): %s",
                    resp->status, resp->message.c_str());
    }

    rclcpp::shutdown();
    return 0;
}
```

## 工具脚本

### 电机状态监控

`scripts/monitor_motors.py` 提供实时电机状态监控功能，可以查看：
- 活跃控制会话（UUID、节点名、控制的电机数）
- 每个电机的位置、速度、力矩
- 电机占用状态（IDLE/OCCUPIED/ERROR）

**使用方法：**

```bash
# 持续监控模式（默认，每秒更新）
python3 scripts/monitor_motors.py

# 单次查询模式
python3 scripts/monitor_motors.py once

# 仅显示电机信息
python3 scripts/monitor_motors.py info
```

**监控界面示例：**

![电机状态监控 - 无活跃会话](docs/images/monitor_1.png)

![电机状态监控 - 有活跃会话](docs/images/monitor_2.png)

---
