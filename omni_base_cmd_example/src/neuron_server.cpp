#include "ros/ros.h"
#include "omni_base_cmd_example/BaseCmd.h"
#include "neuron_example.h"
#include "geometry_msgs/Twist.h"

#define MOVE_ONE_STEP(forward, left, turn, sleep_time) do { \
    msg.linear.x = forward; \
    msg.linear.y = left; \
    msg.angular.z = turn; \
    _pub.publish(msg); \
    ros::Duration(sleep_time).sleep(); \
} while(0);

class NeuronCmdServer {
public:
    NeuronCmdServer() : _nh("~") {
        _ser = _nh.advertiseService(NEURON_CMD_TOPIC, &NeuronCmdServer::neuron_cmd_server, this);
        _pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        _nh.param("velocity", _velocity, 0.5);
        _nh.param("loop_cnt", _loop_cnt, 2);
        ROS_INFO("Velocity is set to %f.", _velocity);
        ROS_INFO("Ready to accept command from client...");
    }
    bool neuron_cmd_server(omni_base_cmd_example::BaseCmd::Request  &req,
                           omni_base_cmd_example::BaseCmd::Response &res);
private:
    ros::NodeHandle    _nh;
    ros::Publisher     _pub;
    ros::ServiceServer _ser;
    double _velocity;
    int    _loop_cnt;
};

bool NeuronCmdServer::neuron_cmd_server(omni_base_cmd_example::BaseCmd::Request  &req,
                                        omni_base_cmd_example::BaseCmd::Response &res)
{
    ROS_INFO("Command received");
    geometry_msgs::Twist msg;
    ROS_INFO("Request: action=%ld", (long int)req.action);

    switch(req.action) {
        case MOVE_F_B:
            ROS_INFO("Try to move forward and backward...");
            ros::Duration(0.5).sleep();
            for (int i = 0; i < _loop_cnt; i++) {
                MOVE_ONE_STEP( _velocity, 0, 0, 0.5);
                MOVE_ONE_STEP( 0, 0, 0, 0.1);
                MOVE_ONE_STEP(-_velocity, 0, 0, 0.5);
                MOVE_ONE_STEP( 0, 0, 0, 0.1);
            }
            res.result = RET_OK;
            break;
        case MOVE_L_R:
            ROS_INFO("Try to move left and right...");
            ros::Duration(0.5).sleep();
            for (int i = 0; i < _loop_cnt; i++) {
                MOVE_ONE_STEP(0,  _velocity, 0, 0.5);
                MOVE_ONE_STEP( 0, 0, 0, 0.1);
                MOVE_ONE_STEP(0, -_velocity, 0, 0.5);
                MOVE_ONE_STEP( 0, 0, 0, 0.1);
            }
            res.result = RET_OK;
            break;
        case TURN_AROUND:
            ROS_INFO("Try to turn around...");
            ros::Duration(0.5).sleep();
            for (int i = 0; i < _loop_cnt; i++) {
                MOVE_ONE_STEP(0, 0,  _velocity, 0.5);
                MOVE_ONE_STEP( 0, 0, 0, 0.1);
                MOVE_ONE_STEP(0, 0, -_velocity, 0.5);
                MOVE_ONE_STEP( 0, 0, 0, 0.1);
            }
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
    NeuronCmdServer server;
    ros::spin();

    return 0;
}
