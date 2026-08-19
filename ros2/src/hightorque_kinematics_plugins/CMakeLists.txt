cmake_minimum_required(VERSION 3.8)
project(hightorque_kinematics_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hightorque_msgs REQUIRED)
find_package(hightorque_kinematics_core REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/no_op_kinematics_plugin.cpp
  src/head_invert_kinematics_plugin.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  sensor_msgs
  hightorque_msgs
  hightorque_kinematics_core)

pluginlib_export_plugin_description_file(
  hightorque_kinematics_core
  plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include)

install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_dependencies(
  pluginlib
  sensor_msgs
  hightorque_msgs
  hightorque_kinematics_core)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
