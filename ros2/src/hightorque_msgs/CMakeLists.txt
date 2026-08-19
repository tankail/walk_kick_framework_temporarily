cmake_minimum_required(VERSION 3.5)
project(hightorque_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generate messages, services and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/DebugData.msg"
  "msg/ControllerState.msg"
  "msg/HightorqueJoy.msg"
  "msg/MotorControlCommand.msg"
  "msg/SessionStatus.msg"
  # Services
  "srv/ChangeState.srv"
  "srv/Common.srv"
  "srv/ExecuteWaypoint.srv"
  "srv/GetAvailableMotors.srv"
  "srv/ModelOutput.srv"
  "srv/ReleaseControl.srv"
  "srv/ResetZero.srv"
  "srv/RequestControl.srv"
  "srv/SwitchPolicy.srv"
  # Actions
  "action/Temp.action"
  DEPENDENCIES std_msgs sensor_msgs
)

ament_package()
