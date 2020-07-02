from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    urdf = Path(get_package_share_directory('omnibot_base'), 'urdf', 'omni.urdf')
    assert urdf.is_file()
    hardware_config = Path(get_package_share_directory('omnibot_base'), 'config', 'hardware.yaml')
    assert hardware_config.is_file()

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        Node(
            package='ydlidar',
            node_executable='ydlidar_node',
            output='screen',
            parameters=[hardware_config],
            ),
    
    
        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            output='screen',
            arguments=[str(urdf)],
            parameters=[hardware_config]
         ),
        ])
