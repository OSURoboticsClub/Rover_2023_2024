#!/bin/bash
modprobe can
modprobe can_raw
modprobe mttcan
modprobe v4l2loopback video_nr=64
ip link set can0 down
sleep 10
ip link set can_arm down
sleep 3
ip link set can_cam down
sleep 3
ip link set can_arm txqueuelen 150
sleep 3
ip link set can_cam txqueuelen 150
sleep 3
ip link set can_arm up type can bitrate 1000000
sleep 3
ip link set can0 up type can bitrate 1000000
sleep 3
ip link set can_cam up type can bitrate 1000000
sleep 3
echo "CAN interface configured"
