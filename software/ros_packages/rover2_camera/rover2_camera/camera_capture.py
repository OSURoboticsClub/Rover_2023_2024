import sys
import threading
import gi
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from gi.repository import Gst, GLib

gi.require_version('Gst', '1.0')
Gst.init(sys.argv)


class CameraCaptureNode(Node):
    def __init__(self):
        super().__init__('camera_capture_node')
        self.get_logger().info('Infrared Stream Node has started!')

        self.declare_parameter('device', '/dev/rover/camera_infrared')
        self.declare_parameter('cap_width', 1920)
        self.declare_parameter('cap_height', 1080)
        self.declare_parameter('cap_framerate', 30)
        self.declare_parameter('preset_level', 1)
        self.declare_parameter('bitrate', 4000000)
        self.declare_parameter('stream_width', 640)
        self.declare_parameter('stream_height', 480)
        self.declare_parameter('fec_percentage', 30)
        self.declare_parameter('udp_host', '192.168.1.1')
        self.declare_parameter('udp_port', 42067)

        self.pipeline = None
        self.loop = None
        self.gst_thread = None

        self.add_on_set_parameters_callback(self.camera_param_update)

        self.start_pipeline()

    def camera_param_update(self, params):
        self.get_logger().info("Parameter update detected, restarting pipeline...")
        if self.loop and self.loop.is_running():
            self.loop.quit()
        self.stop_pipeline()
        self.start_pipeline()
        return SetParametersResult(successful=True)

    def start_pipeline(self):
        device = self.get_parameter('device').value
        cap_width = self.get_parameter('cap_width').value
        cap_height = self.get_parameter('cap_height').value
        cap_framerate = self.get_parameter('cap_framerate').value
        preset_level = self.get_parameter('preset_level').value
        bitrate = self.get_parameter('bitrate').value
        stream_width = self.get_parameter('stream_width').value
        stream_height = self.get_parameter('stream_height').value
        fec_percentage = self.get_parameter('fec_percentage').value
        udp_host = self.get_parameter('udp_host').value
        udp_port = self.get_parameter('udp_port').value

        pipeline_str = (
            f'v4l2src do-timestamp=true device={device} ! '
            f'image/jpeg,width={cap_width},height={cap_height},framerate={cap_framerate}/1 ! '
            f'nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw ! tee name=t '
            f't. ! queue ! appsink name=infrared_sink '
            f't. ! queue ! videoscale ! '
            f'video/x-raw,width={stream_width},height={stream_height},format=I420 ! '
            f'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
            f'nvv4l2h265enc preset-level={preset_level} bitrate={bitrate} ! '
            f'h265parse ! rtph265pay config-interval=1 ! '
            f'rtpulpfecenc percentage={fec_percentage} ! '
            f'udpsink host={udp_host} port={udp_port} sync=false'
        )

        self.get_logger().info(f'Launching pipeline:\n{pipeline_str}')
        self.pipeline = Gst.parse_launch(pipeline_str)

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_message)

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Unable to set pipeline to PLAYING")

        # Create and start GLib main loop in a separate thread
        self.loop = GLib.MainLoop()
        self.gst_thread = threading.Thread(target=self.loop.run, daemon=True)
        self.gst_thread.start()

        self.get_logger().info("Pipeline running in background thread.")

    def stop_pipeline(self):
        if self.pipeline:
            self.get_logger().info("Stopping pipeline...")
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
        if self.loop and self.loop.is_running():
            self.loop.quit()
        if self.gst_thread:
            self.gst_thread.join(timeout=1)
            self.gst_thread = None

    def on_message(self, bus, message):
        msg_type = message.type
        if msg_type == Gst.MessageType.EOS:
            self.get_logger().warn("End of stream received.")
            self.loop.quit()
        elif msg_type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.get_logger().error(f"GStreamer Error: {err}")
            self.loop.quit()
        elif msg_type == Gst.MessageType.WARNING:
            warn, debug = message.parse_warning()
            self.get_logger().warn(f"GStreamer Warning: {warn}")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = CameraCaptureNode()

    try:
        # Let ROS 2 handle parameter updates, etc.
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt: shutting down.")
    finally:
        node.stop_pipeline()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
