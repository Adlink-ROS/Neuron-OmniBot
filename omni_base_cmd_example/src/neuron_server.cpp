#include "ros/ros.h"
#include "omni_base_cmd_example/BaseCmd.h"
#include "neuron_example.h"

bool neuron_cmd_server(omni_base_cmd_example::BaseCmd::Request  &req,
          omni_base_cmd_example::BaseCmd::Response &res)
{
    ROS_INFO("Request: action=%ld", (long int)req.action);

    switch(req.action) {
        case MOVE_F_B:
            ROS_INFO("Try to move forward and backward...");
            res.result = RET_OK;
            break;
        case MOVE_L_R:
            ROS_INFO("Try to move left and right...");
            res.result = RET_OK;
            break;
        case TURN_AROUND:
            ROS_INFO("Try to turn around...");
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
