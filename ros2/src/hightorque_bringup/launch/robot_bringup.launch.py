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
            DeclareLaunchArgument("waist", default_value="0", description="腰部自由度"),
            DeclareLaunchArgument("default_timeout_ms", default_value="200", description="电机超时时间(ms)"),
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context):
    # 获取 launch 参数
    robot_id = context.launch_configurations["robot_id"]
    devel_level = context.launch_configurations["devel_level"]
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

    # 获取 bringup 包路径（配置文件集中存放处）
    bringup_pkg_share = get_package_share_directory("hightorque_bringup")

    # 构建 robot_type 字符串
    robot_type = f"{leg_type}-{leg}L{arm}A{claw}G{head}H{waist}W"
    full_robot_type = f"{model_type}-{robot_type}"

    # robot_config_path 指向 bringup 包中的配置目录
    robot_config_path = os.path.join(bringup_pkg_share, "config", "robot_config", full_robot_type)

    # 配置文件路径
    pd_config_file = os.path.join(robot_config_path, "pd.yaml")
    rl_config_file = os.path.join(robot_config_path, "rl.yaml")
    robot_param_file = os.path.join(robot_config_path, "robot_param.yaml")

    # 动作库路径配置
    home_dir = os.path.expanduser("~")
    policy_path = os.path.join(home_dir, "action_library", "policy_file")
    waypoint_library_path = os.path.join(home_dir, "action_library", "action_config", "waypoint")
    policy_library_path = os.path.join(home_dir, "action_library", "action_config", "policy")
    action_config_path = robot_config_path

    # 获取其他包路径
    try:
        humanoid_pkg_share = get_package_share_directory("humanoid_driver")
        humanoid_config_file = os.path.join(humanoid_pkg_share, "config", "params.yaml")
    except Exception:
        print("Warning: humanoid_driver package not found!")
        humanoid_config_file = ""

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

    # ==================== 主控制节点 ====================
    sim2real_master_node = Node(
        package="sim2real_master",
        executable="sim2real_master_node",
        namespace=namespace,
        output="screen",
        sigterm_timeout="20",
        parameters=[
            {
                "robot_param": robot_param_file,
                "robot_param_path": robot_config_path,
                "model_type": model_type,
                "model_name": "PiPlus",
                "robot_type": robot_type,
                "production_type": "custom",
                "devel_level": devel_level,
                "policy_change_default_policy": "amp",
                "use_fell_down_policy": True,
                "use_fell_down_action": True,
                "robot_config_path": robot_config_path + "/",
                "pd_config_file": pd_config_file,
                "rl_config_file": rl_config_file,
                "policy_file_path": policy_path + "/",
                "policy_config_path": policy_library_path + "/",
                "waypoint_config_path": waypoint_library_path + "/",
            }
        ],
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
        remappings=[("joy", "joy_input")],
        emulate_tty=True,
    )

    # ==================== 手柄映射节点 ====================
    joy_mapper_node = Node(
        package="sim2real_master",
        executable="joy_mapper_node.py",
        namespace=namespace,
        output="screen",
        emulate_tty=True,
    )

    # ==================== 人形机器人驱动节点 ====================
    humanoid_driver_node = Node(
        package="humanoid_driver",
        executable="humanoid_driver",
        namespace=namespace,
        output="screen",
        parameters=[humanoid_config_file],
        emulate_tty=True,
    )

    # ==================== 调试数据记录节点 ====================
    debug_data_logger_node = Node(
        package="sim2real_master",
        executable="debug_data_logger.py",
        namespace=namespace,
        output="screen",
        parameters=[
            {"save_path": os.path.expanduser("~/sim2real_logs")},
            {"enable_logging": True},
        ],
        emulate_tty=True,
    )

    return [
        hightorque_midware_node,
        yesense_imu_node,
        oled_node,
        power_node,
        joy_node,
        joy_mapper_node,
        humanoid_driver_node,
        # sim2real_master_node,
        debug_data_logger_node,
    ]
