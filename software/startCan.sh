#!/bin/bash
modprobe can
modprobe can_raw
modprobe mttcan
modprobe v4l2loopback video_nr=64
ip link set can0 down
sleep 1
ip link set can1 down
sleep 1
ip link set can1 txqueuelen 150
sleep 1
ip link set can1 up type can bitrate 1000000
sleep 1
ip link set can0 up type can bitrate 1000000
sleep 1

echo "CAN interface configured"
