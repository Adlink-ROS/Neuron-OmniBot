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

    ros::NodeHandle    n;
    ros::ServiceClient client;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omni_base_cmd_client");
    if (argc != 2) {
        ROS_ERROR("Usage: rosrun omni_base_cmd_example neuron_client [action]");
        ROS_ERROR("Action 1: Move forward and backward");
        ROS_ERROR("Action 2: Move left and right");
        ROS_ERROR("Action 3: Turn Around");
        return 1;
    }

    Neuron_CMD cmd;
    switch (atoi(argv[1])) {
        case 1:
            cmd.move_forward_backward();
            break;
        case 2:
            cmd.move_left_right();
            break;
        case 3:
            cmd.turn_around();
            break;
        default:
            ROS_ERROR("Wrong Action");
            break;
    }

    return 0;
}
