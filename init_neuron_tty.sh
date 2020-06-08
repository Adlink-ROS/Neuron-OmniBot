#!/bin/bash
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", MODE:="0666", GROUP:="dialout",  SYMLINK+="neurontty"' >/etc/udev/rules.d/neuron_tty.rules
status=$?
[ $status -eq 0 ] && echo "Initialized NeuronBot successfully" || (echo "Failed to initilize NeuronBot!"; exit -1)

sudo udevadm control --reload
sudo udevadm trigger

status=$?
[ $status -eq 0 ] && echo "Udev restarted successfully." || echo "Failed to restart udev!"
