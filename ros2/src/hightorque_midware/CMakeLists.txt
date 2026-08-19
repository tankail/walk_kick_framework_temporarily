cmake_minimum_required(VERSION 3.5)
project(hightorque_midware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hightorque_robot REQUIRED)
find_package(hightorque_kinematics_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hightorque_msgs REQUIRED)

file(GLOB_RECURSE MOTOR_SRC "src/motor/*.cpp")

add_library(motor_sdk STATIC ${MOTOR_SRC})

target_include_directories(motor_sdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${hightorque_robot_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(motor_sdk
    ${Boost_LIBRARIES}
    yaml-cpp
    hightorque_robot::hightorque_robot
    Eigen3::Eigen
)

ament_target_dependencies(motor_sdk
    rclcpp
)

add_executable(hightorque_midware_node
    src/main.cpp
    src/motor_manager_node.cpp
)

target_include_directories(hightorque_midware_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${hightorque_robot_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(hightorque_midware_node
    motor_sdk
    ${Boost_LIBRARIES}
    yaml-cpp
    hightorque_robot::hightorque_robot
    Eigen3::Eigen
)

ament_target_dependencies(hightorque_midware_node
    rclcpp
    pluginlib
    std_msgs
    sensor_msgs
    hightorque_msgs
    hightorque_kinematics_core
)

install(TARGETS motor_sdk
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS hightorque_midware_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

# 安装Python脚本
install(PROGRAMS
    scripts/move_to_zero.py
    scripts/joy_mapper_node.py
    scripts/test_single_motor.py
    scripts/test_joints_individually.py
    scripts/test_all_motors.py
    scripts/monitor_motors.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(motor_sdk)
ament_export_dependencies(
    Eigen3
    Boost
    hightorque_robot
    hightorque_kinematics_core
    hightorque_msgs
    pluginlib
    rclcpp
    std_msgs
    sensor_msgs
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
