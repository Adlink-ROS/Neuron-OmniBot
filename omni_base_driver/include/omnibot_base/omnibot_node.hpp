// Copyright 2019 ADLINK Technology Ltd. Advanced Robotic Platform Group
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OMNIBOT_BASE__OMNIBOT_NODE_HPP_
#define OMNIBOT_BASE__OMNIBOT_NODE_HPP_

#include <chrono>
#include <memory>
#include <algorithm>

#include "omnibot_base/visbility_control.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "base_controller/controllers.hpp"

#define RADIANS(__theta__) ((__theta__)*(3.14159265)/(180))

namespace omnibot_base
{
  class OmniBotNode : public rclcpp::Node {
    public:
      OMNIBOT_BASE_PUBLIC
      explicit OmniBotNode(const rclcpp::NodeOptions & options);
      ~OmniBotNode();
      void param_init(void);
      void cmd_vel_routine_init(void);
      void odom_routine_init(void);
      void imu_routine_init(void);
    private:
      float _vel_gain = 70;
      float _omg_gain = 500;
      double x_e = 0;
      double y_e = 0;
      double th = 0;
      double odom_last_seq = 0;
      double last_odom_time;
      double accel_sensitivity = 1.8*9.81; // 2g
      double gyro_sensitivity = RADIANS(250); // 250deg/sec
      bool enable_tf = true;
      std::string odom_topic;
      std::string imu_topic;
      std::string base_frame_id;
      std::string odom_frame_id;
      std::string imu_frame_id;
      std::shared_ptr<base_controller> controller;
      std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster = nullptr;
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
      rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
      rclcpp::TimerBase::SharedPtr cmd_vel_timer;
  };
}


#endif /* OMNIBOT_BASE__OMNIBOT_NODE_HPP_ */
