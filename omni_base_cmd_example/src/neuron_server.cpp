#include "ros/ros.h"
#include "omni_base_cmd_example/BaseCmd.h"
#include "neuron_example.h"

bool echo(omni_base_cmd_example::BaseCmd::Request  &req,
          omni_base_cmd_example::BaseCmd::Response &res)
{
    res.result = req.action;
    ROS_INFO("Request: type=%ld, action=%ld", (long int)req.type, (long int)req.action);
    ROS_INFO("Response: result=%ld", (long int)res.result);
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "omni_base_cmd_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService(NEURON_CMD_TOPIC, echo);
  ROS_INFO("Ready to accept command from client...");
  ros::spin();

  return 0;
}
