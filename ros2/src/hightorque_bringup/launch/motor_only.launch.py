#!/usr/bin/env python3
"""
简化的机器人启动文件，用于快速测试
只启动电机管理器和基础节点
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 启用 ROS2 彩色日志输出
    os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_id", default_value="", description="机器人ID，用于多机器人部署。留空则不使用namespace"
            ),
            DeclareLaunchArgument(
                "model_type",
                default_value="pi_plus",
                description="模型类型: pi|pi_plus|hi",
            ),
            DeclareLaunchArgument("leg_type", default_value="S", description="腿类型"),
            DeclareLaunchArgument("leg", default_value="12", description="腿自由度"),
            DeclareLaunchArgument("arm", default_value="8", description="手臂自由度"),
            DeclareLaunchArgument("claw", default_value="0", description="夹爪"),
            DeclareLaunchArgument("head", default_value="2", description="头部自由度"),
            DeclareLaunchArgument("waist", default_value="0", description="腰部自由度"),
            DeclareLaunchArgument("default_timeout_ms", default_value="200", description="电机超时时间(ms)"),
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context):
    # 获取 launch 参数
    robot_id = context.launch_configurations["robot_id"]
    model_type = context.launch_configurations["model_type"]
    leg_type = context.launch_configurations["leg_type"]
    leg = context.launch_configurations["leg"]
    arm = context.launch_configurations["arm"]
    claw = context.launch_configurations["claw"]
    head = context.launch_configurations["head"]
    waist = context.launch_configurations["waist"]
    default_timeout_ms = context.launch_configurations["default_timeout_ms"]

    # 构建 namespace（如果提供了 robot_id）
    namespace = f"robot_{robot_id}" if robot_id else ""

    # 获取 bringup 包路径
    bringup_pkg_share = get_package_share_directory("hightorque_bringup")

    # 构建 robot_type 字符串
    robot_type = f"{leg_type}-{leg}L{arm}A{claw}G{head}H{waist}W"
    full_robot_type = f"{model_type}-{robot_type}"

    # robot_config_path 指向 bringup 包中的配置目录
    robot_config_path = os.path.join(bringup_pkg_share, "config", "robot_config", full_robot_type)
    robot_param_file = os.path.join(robot_config_path, "robot_param.yaml")

    # 电机管理节点
    hightorque_midware_node = Node(
        package="hightorque_midware",
        executable="hightorque_midware_node",
        name="hightorque_midware_node",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "robot_param_file": robot_param_file,
                "robot_config_path": robot_config_path,
                "robot_type": full_robot_type,
                "default_timeout_ms": int(default_timeout_ms),
            }
        ],
        emulate_tty=True,
    )

    return [hightorque_midware_node]
