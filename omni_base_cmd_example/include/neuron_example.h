#ifndef _NEURON_EXAMPLE
#define _NEURON_EXAMPLE

#define NEURON_CMD_TOPIC "/neuron_cmd_topic"

enum {
    MOVE_F_B,
    MOVE_L_R,
    TURN_AROUND,
} ACTION;

enum {
    RET_OK = 0,
    RET_WRONG_CMD = -1,
} RESULT;

#endif
