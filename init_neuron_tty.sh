#!/bin/bash
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", MODE:="0666", GROUP:="dialout",  SYMLINK+="neurontty"' >/etc/udev/rules.d/neuron_tty.rules

service udev reload
sleep 2
service udev restart

status=$?
[ $status -eq 0 ] && echo "Initialized successfully" || echo "Failed to initilized!"
