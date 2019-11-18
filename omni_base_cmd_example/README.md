# Introduction
This is a simple example to control NeuronBot with ROS client/server.

# Usage
* Start the server to control NeuronBot
```
rosrun omni_base_cmd_example neuron_server
```
* Send action command to server.
```
# Action 1: Move forward and backward.
# Action 2: Move left and right.
# Action 3: Turn around.
rosrun omni_base_cmd_example neuron_client [action]
```
