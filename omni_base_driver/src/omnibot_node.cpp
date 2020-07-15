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

#include "omnibot_base/omnibot_node.hpp"
#include <cmath>

using namespace std::chrono_literals;
#define CHANGE_ENDIAN_INT16(_x) ((short)((_x & 0xFF00) >> 8 | (_x & 0x00FF) << 8))
//#define RCLCPP_INFO RCLCPP_ERROR
namespace omnibot_base
{
  OmniBotNode::OmniBotNode(const rclcpp::NodeOptions & options) :
    Node("omnibot_node", options)
  {
    this->param_init();
    this->cmd_vel_routine_init();

    /* create odom publisher */
    rclcpp::QoS odom_pub_qos(rclcpp::KeepLast(10));
    this->odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(this->odom_topic, odom_pub_qos);

    /* create imu publisher */
    rclcpp::QoS imu_pub_qos(rclcpp::KeepLast(10));
    this->imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(this->imu_topic, imu_pub_qos);

    /*
     * initial base controller
     * TODO(YuSheng) If there are more controller type,
     * we can select the base controller which are inherit
     * from base_controller class
     */
    std::string device_port;
    uint32_t baudrate = 0;
    this->get_parameter("omnibot_node.device_port", device_port);
    this->get_parameter("omnibot_node.device_baudrate", baudrate);
    RCLCPP_INFO(this->get_logger(), "Opening device %s, baudrate = %d",
        device_port.c_str(), baudrate);
    this->controller = std::make_shared<STMController>(device_port, baudrate);

    try {
      this->controller->init();
      // "R" signal controller to reset
      std::string reset_cmd = "R";
      this->controller->send_cmd(
          std::vector<uint8_t>(reset_cmd.begin(), reset_cmd.end()), reset_cmd.size());

      // "SSSS" signal controller to start
      std::string start_cmd = "SSSS";
      this->controller->send_cmd(
          std::vector<uint8_t>(start_cmd.begin(), start_cmd.end()), start_cmd.size());

    } catch (const std::exception& ex) {
      // TODO(YuSheng) open device failed, should we try to recover here ?
      RCLCPP_ERROR(this->get_logger(), ex.what());
    }

    this->odom_routine_init();
    this->imu_routine_init();
    this->last_odom_time = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::system_clock::now().time_since_epoch()).count();
  }

  OmniBotNode::~OmniBotNode()
  {
    RCLCPP_INFO(this->get_logger(), "shutting down OmniBotNode");
    if (this->controller != nullptr)
    {
      // "R" signal controller to reset
      std::string reset_cmd = "R";
      this->controller->send_cmd(
          std::vector<uint8_t>(reset_cmd.begin(), reset_cmd.end()), reset_cmd.size());
      RCLCPP_INFO(this->get_logger(), "deinit the base controller");
      this->controller->deinit();
    }
  }

  void OmniBotNode::param_init(void)
  {
    /* declare parameters */
    this->declare_parameter("omnibot_node.device_port", "/dev/neurontty",
        rcl_interfaces::msg::ParameterDescriptor());
    this->declare_parameter("omnibot_node.device_baudrate", 115200,
        rcl_interfaces::msg::ParameterDescriptor());
    this->declare_parameter("omnibot_node.baseId", "base_link",
        rcl_interfaces::msg::ParameterDescriptor());
    this->declare_parameter("omnibot_node.odom_topic", "odom",
        rcl_interfaces::msg::ParameterDescriptor());
    this->declare_parameter("omnibot_node.imu_topic", "imu",
        rcl_interfaces::msg::ParameterDescriptor());
    this->declare_parameter("omnibot_node.odom_frame_id", "odom",
        rcl_interfaces::msg::ParameterDescriptor());
    this->declare_parameter("omnibot_node.imu_frame_id", "imu",
        rcl_interfaces::msg::ParameterDescriptor());        
    this->declare_parameter("omnibot_node.enable_tf", false,
        rcl_interfaces::msg::ParameterDescriptor());
    this->declare_parameter("omnibot_node.tx_freq", 5,
        rcl_interfaces::msg::ParameterDescriptor());
    this->declare_parameter("omnibot_node.vel_gain", this->_vel_gain,
        rcl_interfaces::msg::ParameterDescriptor());
    this->declare_parameter("omnibot_node.omg_gain", this->_omg_gain,
        rcl_interfaces::msg::ParameterDescriptor());

    this->get_parameter("omnibot_node.baseId", this->base_frame_id);
    this->get_parameter("omnibot_node.odom_topic", this->odom_topic);
    this->get_parameter("omnibot_node.imu_topic", this->imu_topic);
    this->get_parameter("omnibot_node.odom_frame_id", this->odom_frame_id);
    this->get_parameter("omnibot_node.imu_frame_id", this->imu_frame_id);
    this->get_parameter("omnibot_node.enable_tf", this->enable_tf);
  }

  void OmniBotNode::cmd_vel_routine_init(void)
  {
    /* declare cmd_vel */
    auto cmd_vel_cb =
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
      {
        this->cmd_vel_timer->reset();

        // union to convert endian
        union {
          uint8_t  uint8_val[2];
          int16_t  int16_val;
        } vel_x, vel_y, omega;

        auto buffer = std::vector<uint8_t>(9, 0);

        // float -> int16_t -> uint8_t (big endian)
        vel_x.int16_val = (int16_t) std::max(-32768.0,
            std::min(msg->linear.x * this->_vel_gain, 32768.0));
        vel_y.int16_val = (int16_t) std::max(-32768.0,
            std::min(msg->linear.y * this->_vel_gain, 32768.0));
        omega.int16_val = (int16_t) std::max(-32768.0,
            std::min(msg->angular.z * this->_omg_gain, 32768.0));

        buffer[0] = 0xFF;
        buffer[1] = 0xFE;
        buffer[2] = 0x01;
        buffer[3] = vel_x.uint8_val[1];
        buffer[4] = vel_x.uint8_val[0];
        buffer[5] = vel_y.uint8_val[1];
        buffer[6] = vel_y.uint8_val[0];
        buffer[7] = omega.uint8_val[1];
        buffer[8] = omega.uint8_val[0];

        if (this->controller != nullptr) {
          this->controller->send_cmd(buffer, buffer.size());
        }
      };

    // TODO(YuSheng) Should set deadline qos and it's callback
    this->cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>
      ("cmd_vel", 10, cmd_vel_cb);
    
    auto cmd_vel_timout_cb = 
      [this]() -> void
      {
        // union to convert endian
        const geometry_msgs::msg::Twist::SharedPtr msg;
        auto buffer = std::vector<uint8_t>(9, 0);

        buffer[0] = 0xFF;
        buffer[1] = 0xFE;
        buffer[2] = 0x01;
        buffer[3] = 0x00;
        buffer[4] = 0x00;
        buffer[5] = 0x00;
        buffer[6] = 0x00;
        buffer[7] = 0x00;
        buffer[8] = 0x00;

        if (this->controller != nullptr) {
          this->controller->send_cmd(buffer, buffer.size());
        }
      };
    this->cmd_vel_timer = this->create_wall_timer(1s, cmd_vel_timout_cb);
  }

  void OmniBotNode::odom_routine_init(void)
  {
    // Pre-set message sequence
    controller_event odom_event;
    odom_event.name = "odom";
    odom_event.event_code = 0xFB;
    odom_event.description = "callback when odom message from the base controller";
    odom_event.cb = [this](std::vector<uint8_t> payload_vector) -> void {
      typedef struct
      {
        int16_t x_e;
        int16_t y_e;
        int16_t th_e;
        uint8_t  seq;
      } ODOM_DATA;

      ODOM_DATA *odom_data = (ODOM_DATA *) payload_vector.data();

      static bool first_time = true;
      static uint8_t last_seq = 255;

      if (first_time)
      {
        last_seq = odom_data->seq;
        first_time = false;
      }
      else if ((uint8_t) (odom_data->seq - last_seq) == 1) {
        // RCLCPP_INFO(this->get_logger(), "odom_data->seq correct %d", odom_data->seq);
        last_seq = odom_data->seq;
      }
      else {
        RCLCPP_WARN(this->get_logger(), "odom_data->seq error %d", odom_data->seq);
        last_seq = odom_data->seq;
        return;
      }

      double dx_e = double(CHANGE_ENDIAN_INT16(odom_data->x_e))/10000,
             dy_e = double(CHANGE_ENDIAN_INT16(odom_data->y_e))/10000,
             d_th = double(CHANGE_ENDIAN_INT16(odom_data->th_e))/10000,
             vx =  dx_e * cos(this->th + d_th/2) + dy_e * sin(this->th + d_th/2),
             vy = -dx_e * sin(this->th + d_th/2) + dy_e * cos(this->th + d_th/2);
#if 0
      RCLCPP_INFO(this->get_logger(), "cos = %f, th= %f", cos(this->th + d_th/2), this->th + d_th/2);
      RCLCPP_INFO(this->get_logger(), "sin = %f, th= %f", sin(this->th + d_th/2), this->th + d_th/2);
#endif
      this->x_e += dx_e;
      this->y_e += dy_e;
      this->th  += d_th;

      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.frame_id = this->odom_frame_id;
      odom_msg.child_frame_id = this->base_frame_id;
#if 0
      RCLCPP_INFO(this->get_logger(), "odom_msg.header.frame_id = %s", odom_msg.header.frame_id.c_str());
#endif
      auto now = rclcpp::Clock().now();
      odom_msg.header.stamp = now;
      double now_fsec
        = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::system_clock::now().time_since_epoch()).count();
      double dt = now_fsec - this->last_odom_time;

      odom_msg.pose.pose.position.x = this->x_e;
      odom_msg.pose.pose.position.y = this->y_e;
      odom_msg.pose.pose.position.z = 0;

      tf2::Quaternion quat_tf;
      quat_tf.setRPY(0, 0, this->th);
      geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_tf);
      odom_msg.pose.pose.orientation = quat_msg;

      odom_msg.twist.twist.linear.x = vx / dt;
      odom_msg.twist.twist.linear.y = vy / dt;
      odom_msg.twist.twist.linear.z = 0;

      odom_msg.twist.twist.angular.x = 0;
      odom_msg.twist.twist.angular.y = 0;
      odom_msg.twist.twist.angular.z = d_th / dt;

      *(odom_msg.twist.covariance.begin()) = 1.0;
      *(odom_msg.twist.covariance.end()-1) = 1.0;

      this->odom_pub->publish(std::move(odom_msg));

      if (this->enable_tf) {
        if (this->tf_broadcaster == nullptr) {
          this->tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
        }
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = now;
        transformStamped.header.frame_id = '/'+this->odom_frame_id;
        transformStamped.child_frame_id =  '/'+this->base_frame_id;
        transformStamped.transform.translation.x = this->x_e;
        transformStamped.transform.translation.y = this->y_e;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = quat_tf.x();
        transformStamped.transform.rotation.y = quat_tf.y();
        transformStamped.transform.rotation.z = quat_tf.z();
        transformStamped.transform.rotation.w = quat_tf.w();
        this->tf_broadcaster->sendTransform(transformStamped);
      }

      this->last_odom_time = now_fsec;
#if 0
      // when received odom
      RCLCPP_INFO(this->get_logger(), "received odom data.buffer len = %d", payload_vector.size());
      RCLCPP_INFO(this->get_logger(), "x_e= %x, y_e= %x", 
        CHANGE_ENDIAN_INT16(odom_data->x_e), CHANGE_ENDIAN_INT16(odom_data->y_e));
      RCLCPP_INFO(this->get_logger(), "dx_e= %f, dy_e= %f ", dx_e, dy_e);
      RCLCPP_INFO(this->get_logger(), "position x= %f, y= %f", this->x_e, this->y_e);
      RCLCPP_INFO(this->get_logger(), "vx= %f, vy= %f", vx, vy);
      RCLCPP_INFO(this->get_logger(), "dt=%f", dt);
#endif
    };

    this->controller->register_event(odom_event);
  }

  void OmniBotNode::imu_routine_init(void)
  {
    controller_event imu_event;
    imu_event.name = "imu";
    imu_event.event_code = 0xFA;
    imu_event.description = "callback when imu message from the base controller";
    imu_event.cb = [this](std::vector<uint8_t> payload_vector) -> void {
      typedef struct
      {
        int16_t accel_x;
        int16_t accel_y;
        int16_t accel_z;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
      } IMU_DATA;

      IMU_DATA *imu_data = (IMU_DATA *) payload_vector.data();

      std::unique_ptr<sensor_msgs::msg::Imu> imu_msg =
        std::make_unique<sensor_msgs::msg::Imu>();

      imu_msg->header.frame_id = this->imu_frame_id;
#if 0
      RCLCPP_INFO(this->get_logger(), "imu_msg.header.frame_id = %s", imu_msg->header.frame_id.c_str());
#endif
      auto now = rclcpp::Clock().now();
      imu_msg->header.stamp = now;

      imu_msg->linear_acceleration.x =
        CHANGE_ENDIAN_INT16(imu_data->accel_x) * this->accel_sensitivity / 32768;
      imu_msg->linear_acceleration.y =
        CHANGE_ENDIAN_INT16(imu_data->accel_y) * this->accel_sensitivity / 32768;
      imu_msg->linear_acceleration.z =
        CHANGE_ENDIAN_INT16(imu_data->accel_z) * this->accel_sensitivity / 32768;

      imu_msg->angular_velocity.x =
        CHANGE_ENDIAN_INT16(imu_data->gyro_x) * this->gyro_sensitivity / 32768;
      imu_msg->angular_velocity.y =
        CHANGE_ENDIAN_INT16(imu_data->gyro_y) * this->gyro_sensitivity / 32768;
      imu_msg->angular_velocity.z =
        CHANGE_ENDIAN_INT16(imu_data->gyro_z) * this->gyro_sensitivity / 32768;

      imu_msg->orientation_covariance[0] = -1;

      imu_msg->angular_velocity_covariance[0] = pow(RADIANS(0.05), 2);
      imu_msg->angular_velocity_covariance[4] = pow(RADIANS(0.05), 2);
      imu_msg->angular_velocity_covariance[8] = pow(RADIANS(0.05), 2);

      imu_msg->linear_acceleration_covariance[0] = pow(9.81*400*pow(10, -6), 2);
      imu_msg->linear_acceleration_covariance[4] = pow(9.81*400*pow(10, -6), 2);
      imu_msg->linear_acceleration_covariance[8] = pow(9.81*400*pow(10, -6), 2);

      this->imu_pub->publish(std::move(imu_msg));
    };

    this->controller->register_event(imu_event);
  }

} /* namespace omnibot_base */

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(omnibot_base::OmniBotNode)
