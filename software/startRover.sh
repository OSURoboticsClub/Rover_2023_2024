#!/bin/bash
#sudo modprobe can
#sudo modprobe can_raw
#sudo modprobe mttcan

#sudo ip link set can0 down
#sleep 1
#sudo ip link set can1 down
#sleep 1
#sudo ip link set can1 txqueuelen 150
#sleep 1
#sudo ip link set can1 up type can bitrate 1000000
#sleep 1
#sudo ip link set can0 up type can bitrate 500000
#sleep 1

sudo nvpmodel -m 0
sudo jetson_clocks
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
sudo modprobe v4l2loopback video_nr=64,99,100 exclusive_caps=1
source /home/makemorerobot/Rover_2023_2024/software/install/setup.bash
sleep 15
ros2 launch rover2_main rover2_main_launch.py 2> /home/makemorerobot/Rover_2023_2024/software/output.txt

