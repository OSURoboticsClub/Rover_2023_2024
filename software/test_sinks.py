import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import numpy as np
import time

Gst.init(None)

# Create pipeline
pipeline = Gst.parse_launch(
    "appsrc name=infrared_sink is-live=true format=time ! "
    "videoconvert ! "
    "v4l2sink device=/dev/video64"
)

# Get appsrc element
appsrc = pipeline.get_by_name("infrared_sink")

# Set caps on appsrc
caps = Gst.Caps.from_string("video/x-raw,format=YUY2,width=640,height=480,framerate=30/1")
appsrc.set_property("caps", caps)

# Start pipeline
pipeline.set_state(Gst.State.PLAYING)

# Create a test frame (YUY2 format: 2 bytes per pixel)
width, height = 640, 480
frame = np.zeros((height, width, 2), dtype=np.uint8)
frame[:, :, 0] = 76   # Y value
frame[:, :, 1] = 84   # UV value

# Push frames
print("Pushing frames to /dev/video64...")

while True:  # Push 100 frames
    buf = Gst.Buffer.new_wrapped(frame.tobytes())
    appsrc.emit("push-buffer", buf)
    time.sleep(1/30)  # 30 fps
# Cleanup
pipeline.set_state(Gst.State.NULL)
print("Done")
