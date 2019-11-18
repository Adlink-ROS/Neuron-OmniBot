#include "ros/ros.h"
#include "omni_base_cmd_example/BaseCmd.h"
#include "neuron_example.h"
#include "geometry_msgs/Twist.h"

#define MOVE_ONE_STEP(forward, right, turn, sleep_time) do { \
    msg.linear.x = forward; \
    msg.linear.z = right; \
    msg.angular.z = turn; \
    cmd_vel_pub.publish(msg); \
    ros::Duration(sleep_time).sleep(); \
} while(0);

bool neuron_cmd_server(omni_base_cmd_example::BaseCmd::Request  &req,
          omni_base_cmd_example::BaseCmd::Response &res)
{
    ROS_INFO("Receiving command.....");
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    geometry_msgs::Twist msg;
    ROS_INFO("Request: action=%ld", (long int)req.action);

    switch(req.action) {
        case MOVE_F_B:
            ROS_INFO("Try to move forward and backward...");
            ros::Duration(0.5).sleep();
            MOVE_ONE_STEP( 0.5, 0, 0, 0.5);
            MOVE_ONE_STEP(   0, 0, 0, 0.5);
            MOVE_ONE_STEP(-0.5, 0, 0,   1);
            MOVE_ONE_STEP(   0, 0, 0, 0.5);
            MOVE_ONE_STEP( 0.5, 0, 0, 0.5);
            MOVE_ONE_STEP(   0, 0, 0, 0.5);
            res.result = RET_OK;
            break;
        case MOVE_L_R:
            ROS_INFO("Try to move left and right...");
            ros::Duration(1).sleep();
            MOVE_ONE_STEP(0,  0.5, 0, 0.5);
            MOVE_ONE_STEP(0,    0, 0, 0.5);
            MOVE_ONE_STEP(0, -0.5, 0,   1);
            MOVE_ONE_STEP(0,    0, 0, 0.5);
            MOVE_ONE_STEP(0,  0.5, 0, 0.5);
            MOVE_ONE_STEP(0,    0, 0, 0.5);
            res.result = RET_OK;
            break;
        case TURN_AROUND:
            ROS_INFO("Try to turn around...");
            ros::Duration(1).sleep();
            MOVE_ONE_STEP(0, 0,  0.5, 0.5);
            MOVE_ONE_STEP(0, 0,    0, 0.5);
            MOVE_ONE_STEP(0, 0, -0.5,   1);
            MOVE_ONE_STEP(0, 0,    0, 0.5);
            MOVE_ONE_STEP(0, 0,  0.5, 0.5);
            MOVE_ONE_STEP(0, 0,    0, 0.5);
            res.result = RET_OK;
            break;
        default:
            ROS_INFO("Wrong command");
            res.result = RET_WRONG_CMD;
            break;
    }

    ROS_INFO("Response: result=%ld", (long int)res.result);
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "omni_base_cmd_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService(NEURON_CMD_TOPIC, neuron_cmd_server);
  ROS_INFO("Ready to accept command from client...");
  ros::spin();

  return 0;
}
