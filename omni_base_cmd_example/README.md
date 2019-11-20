# Introduction
This is a simple example to control NeuronBot with ROS client/server.

# Usage
* Start the server to control NeuronBot
  - Run node directly.
```
rosrun omni_base_cmd_example neuron_server
```
  - Or you can run launch file.
```
roslaunch omni_base_cmd_example run_cmd_server.launch
```

* There are two parameters to control. Modify them in `param/cmd_server_param.yaml`.
  - `velocity`: The speed NeuronBot moves
  - `loop_cnt`: Move count

* Send action command to server.
```
# Action 1: Move forward and backward.
# Action 2: Move left and right.
# Action 3: Turn around.
rosrun omni_base_cmd_example neuron_client [action]
```
