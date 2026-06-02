from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(get_package_share_directory('gas_monitor'), 'config', 'gas_params.yaml')
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{name}]: {message}'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        Node(
            package='gas_monitor',
            executable='serial_gas_node',
            name='serial_gas_node',
            output='screen',
            output_format='{line}',
            parameters=[params],
        ),
    ])
