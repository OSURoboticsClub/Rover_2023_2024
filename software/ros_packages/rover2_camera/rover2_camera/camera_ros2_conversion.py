import sys
import threading
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

from rover2_camera_interface.srv import CamParams

Gst = None
GLib = None
CvBridge = None
cv2 = None


def load_camera_libraries():
    global Gst, GLib, CvBridge, cv2
    if Gst is not None:
        return

    from cv_bridge import CvBridge as CvBridgeClass
    import cv2 as cv2_module
    import gi

    gi.require_version('Gst', '1.0')
    from gi.repository import Gst as GstModule, GLib as GLibModule

    GstModule.init(None)
    Gst = GstModule
    GLib = GLibModule
    CvBridge = CvBridgeClass
    cv2 = cv2_module


class CameraCaptureNode(Node):
    def __init__(self):
        super().__init__('camera_capture_node')
        self.get_logger().info('Camera Capture Node has started!')
        load_camera_libraries()

        # Declare parameters
        self.declare_parameter('image_topic', '/camera/d455/color/image_raw')
        self.declare_parameter('cap_width', 640)
        self.declare_parameter('cap_height', 480)
        self.declare_parameter('cap_framerate', 30)
        self.declare_parameter('preset_level', 1)
        self.declare_parameter('bitrate', 4000000)
        self.declare_parameter('stream_width', 640)
        self.declare_parameter('stream_height', 480)
        self.declare_parameter('fec_percentage', 30)
        self.declare_parameter('udp_host', '192.168.1.100')
        self.declare_parameter('udp_port', 42073)
        self.declare_parameter('mux_port', 20000)
        self.declare_parameter('encoder_type', 'nvidia')

        self.bridge    = CvBridge()
        self.pipeline  = None
        self.appsrc    = None
        self.loop      = None
        self.gst_thread = None
        self.pts       = 0
        self.frame_duration = 0

        self.start_pipeline()

        # Subscribe to image topic
        image_topic = self.get_parameter('image_topic').value
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f'Subscribed to {image_topic}')

        # CamParams service
        image_topic_safe = image_topic.replace('/', '_')
        service_name = f'{image_topic_safe}_cam_params'
        self.create_service(CamParams, service_name, self.cam_params_callback)
        self.get_logger().info(f'CamParams service available at: {service_name}')

    def cam_params_callback(self, request, response):
        self.get_logger().info("CamParams service received, updating pipeline...")
        self.set_parameters([
            self.get_parameter_or('preset_level', request.preset_level),
            self.get_parameter_or('bitrate', request.bitrate),
            self.get_parameter_or('stream_width', request.stream_width),
            self.get_parameter_or('stream_height', request.stream_height),
            self.get_parameter_or('fec_percentage', request.fec_percentage),
        ])
        self.restart_pipeline()
        response.ack = "Pipeline restarted with new parameters"
        return response

    def restart_pipeline(self):
        if self.loop and self.loop.is_running():
            self.loop.quit()
        self.stop_pipeline()
        self.start_pipeline()

    def start_pipeline(self):
        cap_width      = self.get_parameter('cap_width').value
        cap_height     = self.get_parameter('cap_height').value
        cap_framerate  = self.get_parameter('cap_framerate').value
        preset_level   = self.get_parameter('preset_level').value
        bitrate        = self.get_parameter('bitrate').value
        stream_width   = self.get_parameter('stream_width').value
        stream_height  = self.get_parameter('stream_height').value
        fec_percentage = self.get_parameter('fec_percentage').value
        udp_host       = self.get_parameter('udp_host').value
        udp_port       = self.get_parameter('udp_port').value
        mux_port       = self.get_parameter('mux_port').value
        encoder_type   = self.get_parameter('encoder_type').value

        self.frame_duration = Gst.SECOND // cap_framerate
        self.pts = 0

        if encoder_type == 'software':
            bitrate_kbps = max(1, int(bitrate) // 1000)
            encoder_pipeline = (
                f'videoconvert ! video/x-raw,format=I420 ! '
                f'x265enc tune=zerolatency speed-preset=ultrafast '
                f'bitrate={bitrate_kbps} key-int-max={cap_framerate} ! '
            )
        else:
            encoder_pipeline = (
                f'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
                f'nvv4l2h265enc preset-level={preset_level} bitrate={bitrate} ! '
            )

        pipeline_str = (
            f'appsrc name=src is-live=true block=false format=time '
            f'caps=video/x-raw,format=BGR,width={cap_width},height={cap_height},framerate={cap_framerate}/1 ! '
            f'videoconvert ! '
            f'tee name=t '
            f't. ! queue max-size-buffers=2 max-size-bytes=0 max-size-time=0 leaky=downstream ! '
            f'rtpvrawpay ! udpsink host=127.0.0.1 port={mux_port} sync=false async=false '
            f't. ! queue max-size-buffers=2 max-size-bytes=0 max-size-time=0 leaky=downstream ! '
            f'videoscale ! video/x-raw,width={stream_width},height={stream_height} ! '
            f'{encoder_pipeline}'
            f'h265parse ! rtph265pay config-interval=1 ! '
            f'rtpulpfecenc percentage={fec_percentage} ! '
            f'udpsink host={udp_host} port={udp_port} sync=false'
        )

        self.get_logger().info(f'Launching pipeline:\n{pipeline_str}')
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except GLib.Error as e:
            raise RuntimeError(f"Failed to create GStreamer pipeline: {e}") from e

        self.appsrc = self.pipeline.get_by_name('src')
        self.appsrc.connect('need-data', self.on_need_data)

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message', self.on_message)

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Unable to set pipeline to PLAYING")

        self.loop = GLib.MainLoop()
        self.gst_thread = threading.Thread(target=self.loop.run, daemon=True)
        self.gst_thread.start()

        self.get_logger().info("Pipeline running in background thread.")

    def stop_pipeline(self):
        if self.pipeline:
            self.get_logger().info("Stopping pipeline...")
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
            self.appsrc = None
        if self.loop and self.loop.is_running():
            self.loop.quit()
        if self.gst_thread:
            self.gst_thread.join(timeout=2)
            self.gst_thread = None

    def on_need_data(self, src, length):
        # appsrc will call this when it wants data — we push on image_callback instead
        pass

    def image_callback(self, msg: Image):
        if self.appsrc is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge conversion failed: {e}")
            return

        cap_width  = self.get_parameter('cap_width').value
        cap_height = self.get_parameter('cap_height').value

        # Resize if needed
        h, w = frame.shape[:2]
        if w != cap_width or h != cap_height:
            frame = cv2.resize(frame, (cap_width, cap_height))

        # Convert to GStreamer buffer
        data = frame.tobytes()
        buf  = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        buf.pts      = self.pts
        buf.duration = self.frame_duration
        self.pts    += self.frame_duration

        ret = self.appsrc.emit('push-buffer', buf)
        if ret != Gst.FlowReturn.OK:
            self.get_logger().warn(f"appsrc push-buffer returned: {ret}")

    def on_message(self, bus, message):
        msg_type = message.type
        if msg_type == Gst.MessageType.EOS:
            self.get_logger().warn("End of stream received.")
            self.loop.quit()
        elif msg_type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.get_logger().error(f"GStreamer Error: {err}")
            self.get_logger().error(f"Debug: {debug}")
            self.loop.quit()
        elif msg_type == Gst.MessageType.WARNING:
            warn, debug = message.parse_warning()
            self.get_logger().warn(f"GStreamer Warning: {warn}")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = CameraCaptureNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("Shutdown requested.")
    finally:
        node.stop_pipeline()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
