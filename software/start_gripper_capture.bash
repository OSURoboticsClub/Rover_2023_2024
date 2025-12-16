#!/bin/bash
while true
do
echo "Starting camera_gripper loopback"
ffmpeg -f v4l2 -framerate 25 -video_size 720x576 -input_format yuyv422 -i /dev/rover/camera_gripper -f v4l2 -pix_fmt yuyv422 /dev/video21
sleep 1
done
