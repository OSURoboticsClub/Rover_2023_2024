#!/bin/bash
modprobe can
modprobe can_raw
modprobe mttcan

ip link set can0 down
sleep 1
ip link set can1 down
sleep 1
ip link set can1 txqueuelen 150
sleep 1
ip link set can1 up type can bitrate 1000000
sleep 1
ip link set can0 up type can bitrate 500000
sleep 1

echo "CAN interface configured"
