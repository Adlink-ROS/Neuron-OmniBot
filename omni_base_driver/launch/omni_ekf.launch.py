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

"""Launch base driver, hardware and ekf."""
import launch

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    base_launch_file_dir = os.path.join(get_package_share_directory('omnibot_base'), 'launch')

    # Launch base driver    
    omni_base_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([base_launch_file_dir, '/omni_base_driver.launch.py'])
        )

    # Launch ydlidar and robot state publisher
    hardware = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([base_launch_file_dir, '/hardware.launch.py'])
        )

    # Launch robot localization (ekf)
    ekf = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([base_launch_file_dir, '/presence.launch.py'])
        )

    return LaunchDescription([omni_base_driver, hardware, ekf])
