cmake_minimum_required(VERSION 3.8)
project(hightorque_power)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PowerDetect.msg"
  "msg/PowerSwitch.msg"
  "msg/VersionInfo.msg"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(
  include
)

# # CAN Driver library
# add_library(hightorque_can_power STATIC
#   src/livelybot_can_driver.cpp
#   src/livelybot_power.cpp
# )
# ament_target_dependencies(hightorque_can_power
#   rclcpp
#   std_msgs
#   sensor_msgs
# )
# target_link_libraries(hightorque_can_power pthread)

# Power node executable
add_executable(power_node 
  src/power_class.cpp
  src/livelybot_can_driver.cpp
  # src/livelybot_power.cpp
)

ament_target_dependencies(power_node
  rclcpp
  std_msgs
  sensor_msgs
)
target_link_libraries(power_node
  pthread
)

# Link messages to targets (Foxy compatible)
rosidl_target_interfaces(power_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install targets
install(TARGETS
  power_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  power_node
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
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs sensor_msgs)
ament_package()
