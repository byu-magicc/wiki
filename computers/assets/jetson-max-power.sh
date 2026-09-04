#!/bin/bash

# place in /home/nvidia/software/jetson-max-power.sh

echo "engaging-hyper-drive"

# /home/nvidia/jetson_clocks.sh
nvpmodel -m 0

echo $?
echo "drinking-coffee-to-stay-awake"

iw dev wlan0 set power_save off

echo $?

#
