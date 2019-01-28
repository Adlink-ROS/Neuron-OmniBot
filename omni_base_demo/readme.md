# OmniBot Demonstration

# Prerequisite
### Hardware
* OmniBot  
    Please install all the necessary packages for the omniBot. See its coresponding [readme](../readme.md)
   
### Intel RealSense
* Intel Realsense SDK 2.0 (librealsense)   
  Source: https://github.com/IntelRealSense/librealsense/releases/tag/v2.17.1  
  Version: SDK 2.0 (2.17.1)  
  Notice: Apt install is recommanded  
  
* Realsense D435 firmware 5.10.13  
  Link: [firmware](https://downloadcenter.intel.com/download/28237/Latest-Firmware-for-Intel-RealSense-D400-Product-Family?v=t) / 
  [tools and manual for Linux](https://www.intel.com/content/www/us/en/support/articles/000028171/emerging-technologies/intel-realsense-technology.html)
  
* Realsense ROS wrapper 2.0 (2.1.3)  
  Link: https://github.com/intel-ros/realsense/releases
  
* rgbd_launch (as required by the realsense 2.1.3)  
  `sudo apt-get install ros-kinetic-rgbd-launch`

### Intel Object Analytics (movidius)
1. [NCSDK](https://github.com/movidius/ncsdk)  
    Install the NCSDK v1.12.01
    Make sure you have environment set, **without ros stuff**
    This step will install coresponding driver and libraries for Movidius NCS to _/opt/movidius_
  
2. [NC APP Zoo](https://github.com/movidius/ncappzoo)  
    Clone the NC APP Zoo to _/opt/movidius_ directly
    * `make all` will fail due to (a) v4l unable to use realsense as video0 
    * Remove ../tensorflow/topcoder_maups that causes build failure
    * ` make compile'
        
3. [Movidius NCS ROS package](https://github.com/intel/ros_intel_movidius_ncs/tree/devel)     
    Install movidius ncs ROS package with branch "devel"  
    ```bash
    # Copy label files from this project to the installation location of NCSDK
    cp ~/catkin_ws/src/ros_intel_movidius_ncs/data/labels/* /opt/movidius/ncappzoo/data/ilsvrc12/
    ```
    
### Testing:  
* movidius_ncs  
 `$ roslaunch realsense2_camera demo_pointcloud.launch`  
 `$ roslaunch movidius_ncs_launch ncs_camera.launch cnn_type:=mobilenetssd input_topic:=/camera/color/image_raw`  
 `$ roslaunch movidius_ncs_launch ncs_stream_detection_example.launch camera_topic:="/camera/color/image_raw"`  
* object_analytics  
 `$ roslaunch realsense2_camera demo_pointcloud.launch`  
 `$ roslaunch object_analytics_launch analytics_movidius_ncs.launch input_points:=/camera/depth/color/points`  
* You can modify _movidius_ncs_launch/ncs_camera.launch_, so that realsense package can activate properly without any additional arguments
    `realsense_ros_camera -> realsense2_camera`
    `arg camera -> realsense`
    `arg cnn_type -> mobilenetssd`
* **When in doubt, unplug and replug everything**  


# Demonstration
Please also check https://github.com/Adlink-ROS/Neuron-OmniBot, branch [v3.0-ncs](https://github.com/Adlink-ROS/adlink_neuronbot/tree/v3.0-ncs)
* Mapping  
  ```bash
  $ roslaunch omni_base_driver omni_base_ekf.launch
  $ roslaunch omni_base_slam omni_base_gmapping.launch
  
  # After complete mapping, save the map
  $ rosrun map_server map_saver -f my_map
  ```  
  Replace the map my_map.pgm and my_map.yaml under [omni_base_slam/map](../omni_base_slam/map)
  
* Host robot AI following  
  `$ roslaunch omni_base_demo ai_tracking_AIO.launch`  

* For AI viewing with detection bounding box  
  `$ roslaunch movidius_ncs_launch ncs_stream_detection_example.launch camera_topic:=/camera/color/image_raw`
 
