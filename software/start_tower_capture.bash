while true
do
echo "Starting camera_main_navigation loopback"
gst-launch-1.0 v4l2src device=/dev/rover/camera_main_navigation ! image/jpeg,width=640,height=480,framerate=30/1 ! jpegdec ! videoconvert ! v4l2sink device=/dev/video22
sleep 1
done
