#!/bin/bash
ffmpeg -loglevel quiet -f v4l2 -i /dev/rover/scimech_cam -frames:v 1 /home/makemorerobot/Rover_2023_2024/software/science_scripts/images/post_image.jpg
feh /home/makemorerobot/Rover_2023_2024/software/science_scripts/images/post_image.jpg
