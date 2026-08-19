# hightorque_msgs

Consolidated ROS2 message, service, and action definitions for the Hightorque robot system.

## Overview

This package contains all message, service, and action definitions used across the Hightorque robot ecosystem, consolidating definitions from:
- `sim2real_msg_ros2` - Simulation to real-world transfer messages
- `hightorque_motor_manager` - Motor control and management interfaces

## Messages

### DebugData.msg
Debug information for system monitoring.

### HightorqueJoy.msg
Joystick/gamepad input data for robot control.

### MotorControlCommand.msg
Motor control commands including position, velocity, torque, and PD gains.

### StateInfo.msg
Robot state information.

## Services

### Common.srv
Common service interface for general operations.

### GetAvailableMotors.srv
Query available motors in the system.

### ModelOutput.srv
Service for model inference output.

### ReleaseControl.srv
Release control of motors with specified release mode (KEEP_STATE, DAMPING_MODE, NO_CONTROL).

### RequestControl.srv
Request control of specific motors with control mode and timeout settings.

## Actions

### Temp.action
Temporary action definition for testing and development.

## Usage

To use these messages in your ROS2 package:

1. Add dependency in `package.xml`:
```xml
<depend>hightorque_msgs</depend>
```

2. Add dependency in `CMakeLists.txt`:
```cmake
find_package(hightorque_msgs REQUIRED)
ament_target_dependencies(your_target hightorque_msgs)
```

3. Include in C++ code:
```cpp
#include "hightorque_msgs/msg/motor_control_command.hpp"
#include "hightorque_msgs/srv/request_control.hpp"
```

4. Import in Python code:
```python
from hightorque_msgs.msg import MotorControlCommand
from hightorque_msgs.srv import RequestControl
```

## Building

```bash
colcon build --packages-select hightorque_msgs
```
