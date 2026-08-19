cmake_minimum_required(VERSION 3.8)
project(hightorque_oled)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(hightorque_power REQUIRED)
find_package(serial_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
)


find_library(SERIAL_LIB serial_cmake)
if(NOT SERIAL_LIB)
  message(WARNING "Serial library not found. You may need to install ros-foxy-serial or libserial-dev")
endif()
# Find libserialport
find_library(SERIALPORT_LIBRARY serialport)
if(NOT SERIALPORT_LIBRARY)
  message(FATAL_ERROR "libserialport not found")
endif()

# OLED library source files
set(OLED_NODE_FILE
    src/sensor_actuator_status.cpp
    src/ip_addr.cpp
    src/oled_interface.cpp
)

# OLED node executable
add_executable(hightorque_oled_node 
  src/oled_run.cpp
  src/sensor_actuator_status.cpp
  src/ip_addr.cpp
  src/oled_interface.cpp
)
ament_target_dependencies(hightorque_oled_node
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  hightorque_power
)
target_link_libraries(hightorque_oled_node
  ${SERIALPORT_LIBRARY}
  ${SERIAL_LIB}
  yaml-cpp
)

# Hardware test executable
add_executable(hightorque_oled_hd_test 
  src/oled_hardware_test.cpp
  src/sensor_actuator_status.cpp
)
ament_target_dependencies(hightorque_oled_hd_test
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  hightorque_power
)
target_link_libraries(hightorque_oled_hd_test
  ${SERIALPORT_LIBRARY}
  ${SERIAL_LIB}
)

# Test motor state publisher executable
add_executable(test_motor_publisher src/test_motor_publisher.cpp)
ament_target_dependencies(test_motor_publisher
  rclcpp
  sensor_msgs
)

# Install targets
install(TARGETS
  hightorque_oled_node
  hightorque_oled_hd_test
  test_motor_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
# ament_export_libraries(hightorque_oled)
ament_export_dependencies(rclcpp sensor_msgs std_msgs tf2 hightorque_power)
ament_package()
