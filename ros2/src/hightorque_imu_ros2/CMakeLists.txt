cmake_minimum_required(VERSION 3.8)
project(yesense_imu_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 查找 yesense_imu 库
find_package(yesense_imu REQUIRED)

# 添加可执行文件
add_executable(yesense_imu_node src/yesense_imu_node.cpp)

# 设置依赖
ament_target_dependencies(yesense_imu_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# 链接 yesense_imu 库
target_link_libraries(yesense_imu_node
  yesense_imu::yesense_imu
)

# 安装可执行文件
install(TARGETS
  yesense_imu_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
