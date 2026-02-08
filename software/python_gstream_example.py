import gi
import sys
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(sys.argv)

# Optimized: Keep everything on GPU (NVMM memory)
pipeline = Gst.parse_launch(
    'v4l2src do-timestamp=true device=/dev/rover/gripper-rgb ! '
    'image/jpeg,width=1920,height=1080,framerate=30/1 ! nvv4l2decoder mjpeg=1  ! nvvidconv ! video/x-raw ! tee name=t '
    't. ! queue  ! appsink name=infrared_sink '
    't. ! queue ! videoscale ! video/x-raw,width=640,height=480,format=I420 ! '
    'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! nvv4l2h265enc preset-level=1 bitrate=4000000 ! '
    'h265parse ! rtph265pay config-interval=1 ! rtpulpfecenc percentage=100 ! udpsink host=192.168.1.1 port=42068 sync=false'
)
def on_message(bus, message):
    t = message.type
    if t == Gst.MessageType.EOS:
        print("End of stream")
        loop.quit()
    elif t == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print(f"Error: {err}")
        loop.quit()
    elif t == Gst.MessageType.WARNING:
        warn, debug = message.parse_warning()
        print(f"Warning: {warn}")
    return True

def main():
    global loop
    
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", on_message)
    
    print("Starting pipeline...")
    ret = pipeline.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        print("ERROR: Unable to set pipeline to PLAYING")
        sys.exit(1)
    
    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        pipeline.set_state(Gst.State.NULL)

if __name__ == "__main__":
    main()
