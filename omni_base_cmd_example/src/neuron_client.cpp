#include "ros/ros.h"
#include "omni_base_cmd_example/BaseCmd.h"
#include "neuron_example.h"
#include <cstdlib>

class Neuron_CMD {
public:
    Neuron_CMD() {
        client = n.serviceClient<omni_base_cmd_example::BaseCmd>(NEURON_CMD_TOPIC);
    }

    int move_forward_backward() {
        ROS_INFO("Send MOVE_FORWARD_BACKWARD request");
        return call_service(MOVE_F_B);
    }
    
    int move_left_right() {
        ROS_INFO("Send MOVE_LEFT_RIGHT request");
        return call_service(MOVE_L_R);
    }

    int turn_around() {
        ROS_INFO("Send TURN_AROUND request");
        return call_service(TURN_AROUND);
    }
    
private:
    int call_service(int action) {
        omni_base_cmd_example::BaseCmd srv;
    
        srv.request.action = action;
        if (client.call(srv)) {
          ROS_INFO("Result: %ld", (long int)srv.response.result);
        } else {
          ROS_ERROR("Calling service error");
          return 1;
        }
    }

    ros::NodeHandle n;
    ros::ServiceClient client;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omni_base_cmd_client");

    Neuron_CMD cmd;
    cmd.move_forward_backward();
    cmd.move_left_right();
    cmd.turn_around();

    return 0;
}
