import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import gi, time
gi.require_version("Gst","1.0")
from gi.repository import Gst
Gst.init(None)

class CameraMuxing(Node):

    def __init__(self):
        super().__init__('camera_muxing')
        self.publisher_ = self.create_publisher(Int32, 'selected_cam_mux', 10)
        timer_period = .5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.num_cams = 2
        self.current_cam = 0
        self.srcPorts = {
            0:20000,
            1:20001
        }




        self.pipeline = Gst.parse_launch(
    f"input-selector name=sel ! "
    f"videoconvert ! "
    f"v4l2sink device=/dev/video64 "
    f"udpsrc port={self.srcPorts[0]} caps = \"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)RAW, sampling=(string)YCbCr-4:2:0, depth=(string)8, width=(string)640, height=(string)480, colorimetry=(string)BT601-5, payload=(int)96, ssrc=(uint)1103043224, timestamp-offset=(uint)1948293153, seqnum-offset=(uint)27904\" ! rtpvrawdepay ! videoconvert ! video/x-raw,format=YUY2 ! tee name=t0 ! queue ! sel.sink_0 t0. ! queue ! fakesink sync=false "
    f"udpsrc port={self.srcPorts[1]} caps = \"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)RAW, sampling=(string)YCbCr-4:2:0, depth=(string)8, width=(string)640, height=(string)480, colorimetry=(string)BT601-5, payload=(int)96, ssrc=(uint)1103043224, timestamp-offset=(uint)1948293153, seqnum-offset=(uint)27904\" ! rtpvrawdepay ! videoconvert ! video/x-raw,format=YUY2 ! tee name=t1 ! queue ! sel.sink_1 t1. ! queue ! fakesink sync=false "
        )


        self.sel = self.pipeline.get_by_name("sel")

        self.get_logger().info("Starting Gstreamer muxing...")
        self.pipeline.set_state(Gst.State.PLAYING)

    def timer_callback(self):
        src_pad = self.sel.get_static_pad(f"sink_{self.current_cam}")
        self.sel.set_property("active-pad", src_pad)
        
        msg = Int32()
        msg.data = self.current_cam
        self.publisher_.publish(msg)

        self.current_cam += 1
        if self.current_cam >= self.num_cams:
            self.current_cam = 0

        

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CameraMuxing()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
