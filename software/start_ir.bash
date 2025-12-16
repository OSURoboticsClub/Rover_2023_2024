#!/bin/bash
while true
do
echo "Starting camera_infrared stream"
gst-launch-1.0 v4l2src device=/dev/video20 !  videoconvert !  video/x-raw,width=1280,height=1024,framerate=30/1 ! videoscale ! video/x-raw,width=640,height=480,framerate=30/1 !   nvvidconv ! nvv4l2h265enc preset-level=1 !   h265parse !   rtph265pay config-interval=1 !   udpsink host=192.168.1.1 port=42067 sync=false
sleep 1
done
