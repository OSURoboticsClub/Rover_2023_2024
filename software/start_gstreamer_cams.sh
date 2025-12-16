#!/bin/bash
sleep 10
/home/makemorerobot/Rover_2023_2024/software/start_ir.bash &
/home/makemorerobot/Rover_2023_2024/software/start_tower.bash &
/home/makemorerobot/Rover_2023_2024/software/start_gripper.bash &
wait
