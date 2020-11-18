#!/bin/bash
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", MODE:="0666", GROUP:="dialout",  SYMLINK+="neurontty"' >/etc/udev/rules.d/neuron_tty.rules
status=$?
if [ $status -eq 0 ]
then
    echo "Initialized NeuronBot successfully"
else
    echo "Failed to initilize NeuronBot!"
    exit 1
fi

# Trigger udev
sudo udevadm control --reload
sudo udevadm trigger
status=$?
if [ $status -eq 0 ]
then
    echo "Udev restarted successfully."
else
    echo "Failed to restart udev!"
    exit 1
fi
