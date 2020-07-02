# Copyright 2019 ADLINK Technology Ltd. Advanced Robotic Platform Group
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

"""Launch base driver in a component container."""
import launch

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

import launch.actions
import launch_ros.actions

def generate_launch_description():
    """Generate launch description with multiple components."""

    stdout_linebuf_envvar = launch.actions.SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    param_substitutions = {
        'omnibot_node.device_port': '/dev/neurontty',
        'omnibot_node.device_baudrate': 115200,
        #'omnibot_node.odomId': 'base_odom',
        'omnibot_node.odomId': 'odom',
        'omnibot_node.odomFrameId': 'odom',
        'omnibot_node.imuId': 'base_imu',
        #'omnibot_node.enable_tf': False
    }

    container = ComposableNodeContainer(
            node_name='omni_base_driver',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='omnibot_base',
                    node_plugin='omnibot_base::OmniBotNode',
                    node_name='omni_driver',
                    parameters=[param_substitutions],
                    remappings=[
                               #('cmd_vel', 'cy/cmd_vel')
                               ]
                #    extra_arguments=[{'omnibot_node.device_baudrate': '115200'}]
			),
                # ComposableNode(
                #     package='composition',
                #     node_plugin='composition::Talker',
                #     node_name='talker'),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
