import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    urdf_path = Path(get_package_share_directory('omni_base_driver'), 'urdf', 'omni.urdf')
    hardware_config = Path(get_package_share_directory('omni_base_driver'),  'config', 'hardware.yaml')

    # Whether to use EKF for multi-sensor fusion
    use_ekf =  LaunchConfiguration('use_ekf', default='false')
    ekf_config = Path(get_package_share_directory('omni_base_driver'), 'config', 'ekf.yaml')

    # Whether to open RealSense D435
    use_camera = LaunchConfiguration('use_camera', default='false')
    camera_config = Path(get_package_share_directory('omni_base_driver'), 'config', 'camera.yaml')

    return LaunchDescription([

        # YDLidar node
        Node(
            package='ydlidar',
            node_executable='ydlidar_node',
            output='screen',
            parameters=[hardware_config],
        ),

        # RealSense node
        Node(
            condition=IfCondition(use_camera),
            package='realsense_node',
            node_executable='realsense_node',
            node_namespace='',
            output='screen',
            parameters=[camera_config]
        ),
        
        # If we use EKF:
        ComposableNodeContainer(
            condition=IfCondition(use_ekf),
            node_name='omni_base_driver',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='omni_base_driver',
                    node_plugin='omnibot_base::OmniBotNode',
                    node_name='omni_driver',
                    parameters=[{
                        'omnibot_node.device_port': '/dev/ttyS2',
                        'omnibot_node.device_baudrate': 115200,
                        'omnibot_node.odom_topic': 'raw_odom',
                        'omnibot_node.imu_topic': 'imu',
                        'omnibot_node.enable_tf': False
                    }],
                    remappings=[
                               #('cmd_vel', 'cy/cmd_vel')
                               ]
			),
            ],
            output='screen',
        ),        
        Node(
            condition=IfCondition(use_ekf),
            package='robot_localization',
            node_executable='ekf_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[("odometry/filtered", "odom")]
        ),

        # If we don't use EKF:
        ComposableNodeContainer(
            condition=UnlessCondition(use_ekf),
            node_name='omni_base_driver',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='omni_base_driver',
                    node_plugin='omnibot_base::OmniBotNode',
                    node_name='omni_driver',
                    parameters=[{
                        'omnibot_node.device_port': '/dev/ttyS2',
                        'omnibot_node.device_baudrate': 115200,
                        'omnibot_node.odom_topic': 'odom',
                        'omnibot_node.imu_topic': 'imu',
                        'omnibot_node.enable_tf': True
                    }],
                    remappings=[
                               #('cmd_vel', 'cy/cmd_vel')
                               ]
			),
            ],
            output='screen',
        ),    

        # Robot state publiser
        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            output='screen',
            arguments=[str(urdf_path)],
            parameters=[hardware_config]
        ),        
    
    ])