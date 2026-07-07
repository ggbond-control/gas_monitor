from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('gas_monitor'), 'config', 'gas_params_default.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the gas_monitor parameter YAML file.',
        ),
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{name}]: {message}'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        Node(
            package='gas_monitor',
            executable='serial_gas_node',
            name='serial_gas_node',
            output='screen',
            output_format='{line}',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
