gst-launch-1.0 udpsrc port=42067 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=200 !   rtph265depay !   h265parse !   queue max-size-buffers=3000 max-size-time=0 max-size-bytes=0 !   avdec_h265 !   videoconvert !   videorate !   video/x-raw,framerate=15/1 !   autovideosink sync=false &
sleep 1
gst-launch-1.0 udpsrc port=42068 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=200 !   rtph265depay !   h265parse !   queue max-size-buffers=3000 max-size-time=0 max-size-bytes=0 !   avdec_h265 !   videoconvert !   videorate !   video/x-raw,framerate=30/1 !   autovideosink sync=false &
sleep 1
gst-launch-1.0 udpsrc port=42069 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=200 !   rtph265depay !   h265parse !   queue max-size-buffers=3000 max-size-time=0 max-size-bytes=0 !   avdec_h265 !   videoconvert !   videorate !   video/x-raw,framerate=15/1 !   autovideosink sync=false &
sleep 1
echo "Starting gripper IP cam loopback"
#Wait for camera to be actually reachable
while ! nc -zv 192.168.1.11 554 >/dev/null 2>&1; do
    echo "Waiting for camera at 192.168.1.11:554..."
    sleep 10
done
echo "Camera accessible, starting ffmpeg..."
sleep 2
ffplay -fflags nobuffer -flags low_delay -framedrop -probesize 32 -analyzeduration 0 rtsp://192.168.1.11:554
#ffmpeg -rtsp_transport tcp -i rtsp://192.168.1.11:554 -vf format=yuv420p -f v4l2 /dev/video10 &
echo "Opening gripper stream"
sleep 2
#gst-launch-1.0 v4l2src device=/dev/video10 ! videoconvert ! fpsdisplaysink
#gst-launch-1.0 udpsrc port=42070 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=200 !   rtph265depay !   h265parse !   queue max-size-buffers=3000 max-size-time=0 max-size-bytes=0 !   avdec_h265 !   videoconvert !   videorate !   video/x-raw,framerate=30/1 !   autovideosink sync=false &
wait
#gst-launch-1.0 udpsrc port=42067 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=200 !   rtph265depay !   h265parse !   queue max-size-buffers=3000 max-size-time=0 max-size-bytes=0 !   avdec_h265 !   videoconvert !   videorate !   video/x-raw,framerate=30/1 !   v4l2sink device=/dev/video10 sync=false &
#gst-launch-1.0 udpsrc port=42068 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=200 !   rtph265depay !   h265parse !   queue max-size-buffers=3000 max-size-time=0 max-size-bytes=0 !   avdec_h265 !   videoconvert !   videorate !   video/x-raw,framerate=30/1 !   v4l2sink device=/dev/video11 sync=false &
#gst-launch-1.0 udpsrc port=42069 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=200 !   rtph265depay !   h265parse !   queue max-size-buffers=3000 max-size-time=0 max-size-bytes=0 !   avdec_h265 !   videoconvert !   videorate !   video/x-raw,framerate=30/1 !   v4l2sink device=/dev/video12 sync=false &
#wait
