import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # 启用 ROS2 彩色日志输出
    os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_id", default_value="", description="机器人ID，用于多机器人部署。留空则不使用namespace"
            ),
            DeclareLaunchArgument("devel_level", default_value="low", description="开发级别: low|high|mix"),
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
            DeclareLaunchArgument("waist", default_value="1", description="腰部自由度"),
            DeclareLaunchArgument("default_timeout_ms", default_value="200", description="电机超时时间(ms)"),
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context):
    # 获取 robot_id 参数
    robot_id = context.launch_configurations["robot_id"]
    namespace = f"robot_{robot_id}" if robot_id else ""
    # 获取 launch 参数
    devel_level = context.launch_configurations["devel_level"]
    model_type = context.launch_configurations["model_type"]
    leg_type = context.launch_configurations["leg_type"]
    leg = context.launch_configurations["leg"]
    arm = context.launch_configurations["arm"]
    claw = context.launch_configurations["claw"]
    head = context.launch_configurations["head"]
    waist = context.launch_configurations["waist"]
    default_timeout_ms = context.launch_configurations["default_timeout_ms"]

    # 获取 bringup 包路径（配置文件集中存放处）
    bringup_pkg_share = get_package_share_directory("hightorque_bringup")

    # 构建 robot_type 字符串
    robot_type = f"{leg_type}-{leg}L{arm}A{claw}G{head}H{waist}W"
    full_robot_type = f"{model_type}-{robot_type}"

    # robot_config_path 指向 bringup 包中的配置目录
    robot_config_path = os.path.join(bringup_pkg_share, "config", "robot_config", full_robot_type)

    # 配置文件路径
    robot_param_file = os.path.join(robot_config_path, "robot_param.yaml")

    try:
        imu_pkg_share = get_package_share_directory("yesense_imu_ros2")
        imu_config_file = os.path.join(imu_pkg_share, "config", "imu_rknn_config.yaml")
    except Exception:
        print("Warning: yesense_imu_ros2 package not found!")
        imu_config_file = ""

    # ==================== 电机管理节点 ====================
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

    # ==================== IMU 传感器节点 ====================
    yesense_imu_node = Node(
        package="yesense_imu_ros2",
        executable="yesense_imu_node",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "config_file": imu_config_file,
                "publish_rate": 100.0,
            }
        ],
        emulate_tty=True,
    )

    # ==================== OLED 显示节点 ====================
    oled_node = Node(
        package="hightorque_oled",
        executable="hightorque_oled_node",
        name="hightorque_oled_node",
        namespace=namespace,
        output="screen",
        parameters=[{"config_file": robot_param_file, "oled_uart_dev_name": "/dev/ttyS4"}],
        emulate_tty=True,
    )

    # ==================== 电源管理节点 ====================
    power_node = Node(
        package="hightorque_power",
        executable="power_node",
        name="power_node",
        namespace=namespace,
        output="screen",
        emulate_tty=True,
    )

    # ==================== 手柄输入节点 ====================
    joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "dev": "/dev/input/js0",
                "deadzone": 1e-3,
                "autorepeat_rate": 0.0,
                "coalesce_interval": 0.05,
            }
        ],
        # remappings=[("joy", "joy_input")],
        emulate_tty=True,
    )

    # ==================== 手柄映射节点 ====================
    joy_mapper_node = Node(
        package="hightorque_midware",
        executable="joy_mapper_node.py",
        namespace=namespace,
        output="screen",
        # remappings=[("joy", "joy_input"), ("joy_msg", "hightorque_joy")],
        remappings=[("joy_msg", "hightorque_joy")],
        emulate_tty=True,
    )

    return [
        hightorque_midware_node,
        yesense_imu_node,
        oled_node,
        power_node,
        joy_node,
        joy_mapper_node,
    ]
