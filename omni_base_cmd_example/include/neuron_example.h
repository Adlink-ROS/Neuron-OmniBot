#ifndef _NEURON_EXAMPLE
#define _NEURON_EXAMPLE

#define NEURON_CMD_TOPIC "neuron_cmd_topic"

enum {
    MOVE = 0,
    TURN_AROUND,
} ACTION;

enum {
    M_FORWARD_BACKWARD = 0,
    M_LEFT_RIGHT,
} MOVE_ACTION;

enum {
    T_LEFT_RIGHT = 0,
} TURN_AROUND_ACTION;

#endif
