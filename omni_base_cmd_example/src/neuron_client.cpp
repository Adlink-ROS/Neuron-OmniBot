#include "ros/ros.h"
#include "omni_base_cmd_example/BaseCmd.h"
#include "neuron_example.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omni_base_cmd_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<omni_base_cmd_example::BaseCmd>(NEURON_CMD_TOPIC);
    omni_base_cmd_example::BaseCmd srv;
    
    ROS_INFO("Send TURN_AROUND request");
    srv.request.type = TURN_AROUND;
    srv.request.action = T_LEFT_RIGHT;
    if (client.call(srv)) {
      ROS_INFO("Result: %ld", (long int)srv.response.result);
    } else {
      ROS_ERROR("Calling service error");
      return 1;
    }

    return 0;
}
