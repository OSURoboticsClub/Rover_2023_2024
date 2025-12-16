#!/bin/bash
while true
do
echo "Starting camera_infrared loopback"
gst-launch-1.0 v4l2src device=/dev/rover/camera_infrared ! image/jpeg,width=1280,height=1024,framerate=30/1 ! jpegdec ! videoconvert ! v4l2sink device=/dev/video20
sleep 1
done
