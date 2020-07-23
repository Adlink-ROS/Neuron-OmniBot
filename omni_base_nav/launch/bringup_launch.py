# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from nav2_common.launch import Node

def generate_launch_description():
    # Get the launch directory
    my_nav_dir = get_package_share_directory('omni_base_nav')
    my_launch_dir = os.path.join(my_nav_dir, 'launch')
    my_param_dir = os.path.join(my_nav_dir, 'param')
    my_param_file = 'omnibot_params.yaml'
    my_bt_file = 'navigate_w_replanning.xml'
    my_map_dir = os.path.join(my_nav_dir, 'map')
    my_map_file = 'mememan.yaml'

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    autostart = LaunchConfiguration('autostart')
    open_rviz = LaunchConfiguration('open_rviz')
    use_remappings = LaunchConfiguration('use_remappings')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(my_map_dir, my_map_file),
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(my_param_dir, my_param_file),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(my_param_dir, my_bt_file),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_remappings_cmd = DeclareLaunchArgument(
        'use_remappings',
        default_value='false',
        description='Arguments to pass to all nodes launched by the file')

    declare_open_rviz_cmd = DeclareLaunchArgument(
        'open_rviz',
        default_value='false',
        description='Launch Rviz?')

    # Specify the actions
    start_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(my_launch_dir, 'localization_launch.py')),
        launch_arguments={'namespace': namespace,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'params_file': params_file,
                          'use_lifecycle_mgr': 'false',
                          'use_remappings': use_remappings}.items())

    start_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(my_launch_dir, 'navigation_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'params_file': params_file,
                          'bt_xml_file': bt_xml_file,
                          'use_lifecycle_mgr': 'false',
                          'use_remappings': use_remappings,
                          'map_subscribe_transient_local': 'true'}.items())
                              
    start_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(my_launch_dir, 'rviz_view_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'open_rviz': open_rviz,
                          'map_subscribe_transient_local': 'true'}.items())
                             
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        node_executable='lifecycle_manager',
        node_name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['map_server',
                                    'amcl',
                                    'controller_server',
                                    'planner_server',
                                    'recoveries_server',
                                    'bt_navigator',
                                    'waypoint_follower']}])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_use_remappings_cmd)
    ld.add_action(declare_open_rviz_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_localization_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_navigation_cmd)

    return ld
