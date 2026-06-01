from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(get_package_share_directory('gas_monitor'), 'config', 'gas_params.yaml')
    return LaunchDescription([
        Node(
            package='gas_monitor',
            executable='serial_gas_node',
            name='serial_gas_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='gas_monitor',
            executable='http_gas_node',
            name='http_gas_node',
            output='screen',
            parameters=[params],
        ),
    ])
