import sys

from rover2_spectrometry_interface.srv import SpectrometryInterface
import rclpy
from rclpy.node import Node


class Spectrometry_Client(Node):

    def __init__(self):
        super().__init__('spectrometry_chart_client')
        self.cli = self.create_client(SpectrometryInterface, 'spectrometry_chart')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpectrometryInterface.Request()

    def send_request(self, camera_id, reaction_type):
        self.req.camera_id = camera_id
        self.req.reaction_type = reaction_type
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    spectrometry_client = Spectrometry_Client()
    future = spectrometry_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin_until_future_complete(spectrometry_client, future)
    response = future.result()
    spectrometry_client.get_logger().info(
        'Success?: %d' %
        response.success)

    spectrometry_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()