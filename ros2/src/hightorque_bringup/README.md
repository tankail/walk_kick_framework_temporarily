# hightorque_bringup

`hightorque_bringup` organizes the core nodes and configurations required for robot runtime. Common uses:

1. Automatically locate configuration directories based on robot configuration
2. Launch midware, IMU, OLED, power, gamepad, and other foundational nodes in one go

The primary launch file is [pi_plus_rknn.launch.py](launch/pi_plus_rknn.launch.py).

## Configuration Directories

Bringup constructs a `robot_type` from the combination of `model_type/leg_type/leg/arm/claw/head/waist`, for example:

```text
pi_plus-S-12L8A0G2H0W
```

The corresponding configuration directory is located at:

```text
hightorque_bringup/config/robot_config/<robot_type>/
```

This directory typically contains at least:

- `robot_param.yaml`: Robot hardware and device-level configuration for low-level drivers and peripheral nodes
- `pd.yaml`: Joint mapping, limits, direction, default `kp/kd` and other control configurations, primarily for midware
- `custom_action.yaml`: Custom action configurations for certain robot configurations

## Significance of pd.yaml

`pd.yaml` is not just a PD parameter table; it's more like a "joint control configuration table". The current `hightorque_midware_node` automatically locates this file via `robot_config_path` and reads its configuration at startup.

Path example:

```text
hightorque_bringup/config/robot_config/pi_plus-S-12L8A0G2H0W/pd.yaml
```

It primarily determines:

- `joint_names`: Joint name order, corresponding to joint semantics in the system
- `map_index`: Mapping between joints and motor IDs
- `direction`: Positive/negative direction correction, determining which direction the motor turns for the same angle command
- `lower/upper`: Joint limits, referenced by midware before sending commands
- `urdf_offset`: Offset between URDF zero position and actual motor zero position
- `kp/kd`: Default control gains
- `supports_kinematics`: Whether to enable forward/inverse kinematics capabilities

### How midware uses pd.yaml

The current `hightorque_midware_node` uses `pd.yaml` to complete these initializations:

- Establish mapping from joint names to motor IDs
- Read default `kp/kd` as fallback parameters for position control
- Read `direction` and `urdf_offset` to align software and hardware coordinates
- Read `lower/upper` as command limiting basis

Think of it this way:

- `robot_param.yaml` leans toward "device and hardware connection"
- `pd.yaml` leans toward "joint control and motion semantics"

## Nodes Launched by Bringup

The following description is based on the current [pi_plus_rknn.launch.py](launch/pi_plus_rknn.launch.py).

### 1. hightorque_midware_node

Package: `hightorque_midware`
Executable: `hightorque_midware_node`

Purpose:

- Manages motor control authority
- Provides `/request_control`, `/release_control`, `/control_command` and other interfaces
- Converts high-level control commands to low-level motor commands
- Publishes `/joint_states` and session status

This is the core "motor control middleware" of the entire system. Upper-level controllers, test scripts, and action programs generally don't directly command motors but go through it first.

### 2. yesense_imu_node

Package: `yesense_imu_ros2`
Executable: `yesense_imu_node`

Purpose:

- Reads IMU device data
- Publishes attitude, angular velocity, acceleration and other inertial navigation information
- Provides torso attitude input for balance control, state estimation, and policy control

Without this node, attitude control and state estimation that depend on IMU typically cannot function properly.

### 3. hightorque_oled_node

Package: `hightorque_oled`
Executable: `hightorque_oled_node`

Purpose:

- Drives the OLED display on the robot body
- Displays device status, running status, or prompt information
- Allows viewing basic status during on-site debugging without connecting to a terminal

This node is essentially a "local status display".

### 4. power_node

Package: `hightorque_power`
Executable: `power_node`

Purpose:

- Manages power-related status
- Reads or publishes battery/power supply information
- Serves as the entry point for robot power-on, power monitoring, low-voltage warnings, etc.

Its specific topics and details depend on the `hightorque_power` package itself, but from bringup's perspective, it's the whole-machine power management node.

### 5. joy_linux_node

Package: `joy_linux`
Executable: `joy_linux_node`

Purpose:

- Reads raw gamepad input from `/dev/input/js0`
- Publishes raw gamepad messages
- The current launch file remaps the default `joy` topic to `joy_input`

It only handles "collecting raw gamepad input", not business semantic conversion.

### 6. joy_mapper_node.py

Package: `hightorque_midware`
Executable: `joy_mapper_node.py`

Purpose:

- Subscribes to `joy_input`
- Converts Linux raw gamepad button/joystick values to the project's unified internal format
- Outputs `hightorque_joy`

Think of it as a "gamepad protocol conversion layer".

### 7. humanoid_driver

Package: `humanoid_driver`
Executable: `humanoid_driver`

Purpose:

- Gamepad filtering

## Understanding the Launch Flow

The current bringup's responsibilities can be summarized as:

1. Locate configuration directory based on robot configuration
2. Launch hardware-related foundational nodes
3. Launch motor control middleware `hightorque_midware_node`
4. Launch gamepad input chain
5. Reserve policy controller integration position

So it's more like a "whole-machine basic runtime environment" rather than a single algorithm node.

## Common Parameters

The current launch file declares these common parameters:

- `robot_id`: Used to add namespace for multi-robot deployment
- `model_type`: Model type, such as `pi`, `pi_plus`, `hi`
- `leg_type/leg/arm/claw/head/waist`: Combine to form `robot_type`
- `default_timeout_ms`: Midware control timeout
- `policy_name`: Policy name for policy controller use

Where:

- When `robot_id` is empty, no namespace is added
- When `robot_id` is not empty, nodes enter the `robot_<robot_id>` namespace

## Example

For example:

```bash
ros2 launch hightorque_bringup pi_plus_rknn.launch.py \
  model_type:=pi_plus \
  leg_type:=S \
  leg:=12 \
  arm:=8 \
  claw:=0 \
  head:=2 \
  waist:=0
```

Will resolve to:

```text
robot_type = pi_plus-S-12L8A0G2H0W
```

Then primarily load:

- `hightorque_bringup/config/robot_config/pi_plus-S-12L8A0G2H0W/robot_param.yaml`
- `hightorque_bringup/config/robot_config/pi_plus-S-12L8A0G2H0W/pd.yaml`

## Debugging Suggestions

If the robot behaves incorrectly after bringup, troubleshoot in this order:

1. First confirm the correct `robot_type` was loaded
2. Check `map_index/direction/urdf_offset/kp/kd` in `pd.yaml`
3. Check if `midware` is properly publishing `/joint_states`
4. Check if IMU, power, joy all started normally
5. Finally check if there are issues at the policy or action layer
