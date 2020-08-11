# Neuron OmniBot

## ROS 2 Instructions

### Install and Build

1. Install ROS 2 [Foxy](https://index.ros.org/doc/ros2/Installation/Foxy/)

2. Git clone this package and others source

```sh
mkdir -p ~/omnibot_ros2_ws/src
cd ~/omnibot_ros2_ws/
wget https://raw.githubusercontent.com/Adlink-ROS/neuron-omnibot_ros2.repos/foxy-devel/neuron-omnibot_ros2.repos
vcs import src < neuron-omnibot_ros2.repos
```

3. Install dependencies

```sh
cd ~/omnibot_ros2_ws/
source /opt/ros/foxy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro foxy
```

4. Colcon build the package

```sh
cd ~/omnibot_ros2_ws/
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/omnibot_ros2_ws/install/local_setup.bash
```

5. Init TTY for OmniBot and YDLidar

```sh
cd ~/omnibot_ros2_ws/src/Neuron-OmniBot/init/
sudo ./omnibot_init.sh
```

### Bringup Base Driver
```sh
source /opt/ros/foxy/setup.bash
source ~/omnibot_ros2_ws/install/local_setup.bash
ros2 launch omni_base_driver bringup_launch.py
```

### Bringup Navigation

For each launch command, make sure you have sourced the local_setup.bash from ROS 2 and OmniBot.

**Option 1**: all-in-one
```sh
ros2 launch omni_base_nav bringup_launch.py map:=<full_path_to_your_map_name.yaml> open_rviz:=true
```

**Option 2**: individual launch files
```sh
ros2 launch omni_base_nav localization_launch.py map:=<full_path_to_your_map_name.yaml>
ros2 launch omni_base_nav navigation_launch.py
ros2 launch omni_base_nav rviz_view_launch.py
```

### Run Navigation and SLAM Simultaneously
```sh
ros2 launch omni_base_nav slam_launch.py
ros2 launch omni_base_nav navigation_launch.py
ros2 launch omni_base_nav rviz_view_launch.py

# For saving a map, run below commandL
ros2 run nav2_map_server map_saver_cli -f <map_dir>/<map_name>
```


