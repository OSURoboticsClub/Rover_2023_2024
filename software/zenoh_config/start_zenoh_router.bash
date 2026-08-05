#!/bin/bash

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CONFIG_URI='/home/makemorerobot/Rover_2023_2024/software/zenoh_config/jetson.json5'

source /opt/ros/humble/setup.bash

ros2 daemon stop

ros2 run rmw_zenoh_cpp rmw_zenohd