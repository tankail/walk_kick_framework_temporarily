import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取功能包的配置目录
    pkg_share = get_package_share_directory('yesense_imu_ros2')
    default_config_file = os.path.join(pkg_share, 'config', 'imu_config.yaml')

    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='IMU配置文件的完整路径'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='IMU数据发布频率(Hz)'
    )

    # 节点
    yesense_imu_node = Node(
        package='yesense_imu_ros2',
        executable='yesense_imu_node',
        name='yesense_imu_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }]
    )

    return LaunchDescription([
        config_file_arg,
        publish_rate_arg,
        yesense_imu_node
    ])
