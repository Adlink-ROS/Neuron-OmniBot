#!/bin/bash

# udev for neurontty, if we have ttyUSB*, then link ttyUSB* as /dev/neurontty
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", MODE:="0666", GROUP:="dialout",  SYMLINK+="neurontty"' > /etc/udev/rules.d/neurontty.rules
status=$?
if [ $status -eq 0 ]
then
    echo "Initialized Neuron-OmniBot successfully"
else
    echo "Failed to initialize Neuron-OmniBot!"
    exit 1
fi

# udev for YDLidar
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar-2303.rules
status=$?
if [ $status -eq 0 ]
then
    echo "Initialized YDLidar successfully"
else
    echo "Failed to initialize YDLidar!"
    exit 1
fi

# Trgiier udev
sudo udevadm control --reload
sudo udevadm trigger
status=$?
if [ $status -eq 0 ]
then
    echo "Udev restarted successfully"
else
    echo "Failed to restart udev!"
    exit 1
fi
