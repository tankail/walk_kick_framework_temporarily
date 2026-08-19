from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Power node
    power_node = Node(
        package='hightorque_power',
        executable='power_node',
        name='power_node',
        output='screen'
    )

    return LaunchDescription([
        power_node
    ])
