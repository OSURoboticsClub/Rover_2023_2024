import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import gi, time
gi.require_version("Gst","1.0")
from gi.repository import Gst
from nav_autonomy_interface.srv import SwitchCamera  

Gst.init(None)

class CameraMuxing(Node):
    def __init__(self):
        super().__init__('camera_muxing')
        self.publisher_ = self.create_publisher(Int32, 'selected_cam_mux', 10)
        self.num_cams = 2   # KRJ TODO: sync this as a single parameter with the yolo node
        self.current_cam = 0
        self.srcPorts = {
            0: 20002,
            1: 20003
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

        self.srv = self.create_service(SwitchCamera, 'switch_camera', self.switch_camera_callback)

    def switch_camera_callback(self, request, response):
        cam_idx = request.cam_idx
        if cam_idx >= self.num_cams:
            response.ack = SwitchCamera.Response.FAIL
            return response

        self.current_cam = cam_idx
        src_pad = self.sel.get_static_pad(f"sink_{self.current_cam}")
        self.sel.set_property("active-pad", src_pad)
        print(self.current_cam)

        response.ack = SwitchCamera.Response.SUCCESS
        return response


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = CameraMuxing()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()