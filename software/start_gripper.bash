#!/bin/bash
while true
do
echo "Starting camera_infrared stream"
gst-launch-1.0 v4l2src device=/dev/video21 !  videoconvert !  video/x-raw,width=720,height=576,framerate=30/1 !   nvvidconv ! nvv4l2h265enc preset-level=1 !   h265parse !   rtph265pay config-interval=1 !   udpsink host=192.168.1.1 port=42068 sync=false 

sleep 1
done
