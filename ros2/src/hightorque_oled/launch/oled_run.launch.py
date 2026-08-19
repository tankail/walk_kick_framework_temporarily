from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='config/robot_pi_12dof_cfg.yaml)'
    )

    oled_uart_dev_name_arg = DeclareLaunchArgument(
        'oled_uart_dev_name',
        default_value='/dev/ttyACM',
        description='OLED UART device name'
    )

    # Get config file path
    # config_file = LaunchConfiguration('config_file')

    # Power node
    power_node = Node(
        package='hightorque_power',
        executable='power_node',
        name='power_node',
        output='screen',
    )

    # OLED node - load both config file and specific parameters
    oled_node = Node(
        package='hightorque_oled',
        executable='hightorque_oled_node',
        name='hightorque_oled_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'oled_uart_dev_name': LaunchConfiguration('oled_uart_dev_name')
        }]
    )

    return LaunchDescription([
        config_file_arg,
        oled_uart_dev_name_arg,
        power_node,
        oled_node
    ])
