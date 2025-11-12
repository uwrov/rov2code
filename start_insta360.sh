#!/bin/bash
sudo modprobe -a usbip-core usbip-host
sudo usbipd -D
id=$(usbip list -l | grep 2e1a | head -1 | awk '{ split($0,a," "); print a[3] }')
echo "Attempting to bind busid $id..."
sudo usbip bind -b $id
