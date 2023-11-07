#!/bin/bash


# For Jetson CAN info: https://docs.nvidia.com/jetson/archives/r35.2.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html
# Generic CAN setup for Linux:
# After connecting to CAN, check if CAN device is connected.
#     $ ip addr | grep "can"
# Load the SocketCAN kernel modules
#   CAN BUS subsystem support module:
#     $ sudo modprobe can
#   Raw CAN protocol module:
#     $ sudo modprobe can_raw
# Verify if kernel modules are loaded properly
#     $ lsmod | grep "can"

sudo apt-get install can-utils

sudo modprobe can
sudo modprobe can_raw

if lsmod | grep -q "can"; then
  ip link set can0 type can bitrate 500000 loopback on
  ip link set can0 up
  candump can0 &
  cansend can0 123#abcdabcd
fi



