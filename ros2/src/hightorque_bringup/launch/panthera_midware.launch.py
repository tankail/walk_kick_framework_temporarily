#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def generate_launch_description():
    # 启用 ROS2 彩色日志输出
    os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_id", default_value="", description="机器人ID，用于多机器人部署。留空则不使用namespace"
            ),
            DeclareLaunchArgument(
                "role", default_value="leader", description="角色: leader|follower，也兼容 master|slave"
            ),
            DeclareLaunchArgument("dof", default_value="6", description="自由度: 6|7"),
            DeclareLaunchArgument(
                "model_type",
                default_value="common",
                description="模型类型:common",
            ),
            DeclareLaunchArgument("default_timeout_ms", default_value="200", description="电机超时时间(ms)"),
            OpaqueFunction(function=launch_setup),
        ]
    )


def normalize_role(role: str) -> str:
    value = role.strip().lower()
    if value in ("leader", "master"):
        return "leader"
    if value in ("follower", "slave"):
        return "follower"
    raise RuntimeError(f"Unsupported role: {role}. Expected leader/follower/master/slave")


def resolve_config_name(role: str, dof: str) -> str:
    normalized_role = normalize_role(role)
    if dof not in ("6", "7"):
        raise RuntimeError(f"Unsupported dof: {dof}. Expected 6 or 7")

    # 历史配置目录命名不一致：leader 使用 panthear，follower 使用 panthera。
    if normalized_role == "leader":
        return f"panthear-{dof}-leader"
    return f"panthera-{dof}-follower"


def launch_setup(context):
    robot_id = context.launch_configurations["robot_id"]
    role = context.launch_configurations["role"]
    dof = context.launch_configurations["dof"]
    model_type = context.launch_configurations["model_type"]
    default_timeout_ms = context.launch_configurations["default_timeout_ms"]

    namespace = f"robot_{robot_id}" if robot_id else ""

    bringup_pkg_share = get_package_share_directory("hightorque_bringup")
    config_name = resolve_config_name(role, dof)
    robot_config_path = os.path.join(bringup_pkg_share, "config", "panthera", config_name)
    robot_param_file = os.path.join(robot_config_path, "robot_param.yaml")

    if not os.path.exists(robot_param_file):
        raise RuntimeError(f"robot_param.yaml not found: {robot_param_file}")

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
                "robot_type": config_name,
                "model_type": model_type,
                "default_timeout_ms": int(default_timeout_ms),
            }
        ],
        emulate_tty=True,
    )

    return [hightorque_midware_node]
