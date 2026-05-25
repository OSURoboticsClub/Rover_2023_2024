import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
Gst.init(None)

class ImageStreamer(Node):
    def __init__(self):
        super().__init__('image_streamer')
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('image_topic', '/camera/d455/color/image_raw')
        self.declare_parameter('host', '127.0.0.1')  # receiver IP
        self.declare_parameter('port', 5000)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        
        topic = self.get_parameter('image_topic').value
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        
        # Subscribe to ROS2 image topic
        self.sub = self.create_subscription(Image, topic, self.callback, 10)
        
        # GStreamer UDP pipeline
        pipeline_str = (
            f"appsrc name=source is-live=true format=time "
            f"! video/x-raw,format=BGR,width={self.width},height={self.height},framerate={self.fps}/1 "
            f"! videoconvert "
            f"! x264enc tune=zerolatency bitrate=2000 speed-preset=superfast "
            f"! rtph264pay config-interval=1 pt=96 "
            f"! udpsink host={host} port={port} sync=false"
        )
        
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsrc = self.pipeline.get_by_name('source')
        
        # Set caps on appsrc
        caps = Gst.Caps.from_string(f"video/x-raw,format=BGR,width={self.width},height={self.height},framerate={self.fps}/1")
        self.appsrc.set_property('caps', caps)
        
        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info(f"Streaming {topic} → udp://{host}:{port}")
    
    def callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        data = frame.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        self.appsrc.emit('push-buffer', buf)

def main(args=None):
    rclpy.init(args=args)
    node = ImageStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.set_state(Gst.State.NULL)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
