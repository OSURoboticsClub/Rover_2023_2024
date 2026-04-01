import sys

from rgb_interfaces.srv import Rgb
import rclpy
from rclpy.node import Node


class RGB_Client(Node):

    def __init__(self):
        super().__init__('rgb_chart_client')
        self.cli = self.create_client(Rgb, 'rgb_chart')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Rgb.Request()

    def send_request(self, camera_id, reaction_type):
        self.req.camera_id = camera_id
        self.req.reaction_type = reaction_type
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    rgb_client = RGB_Client()
    future = rgb_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin_until_future_complete(rgb_client, future)
    response = future.result()
    rgb_client.get_logger().info(
        'Success?: %d' %
        response.success)

    rgb_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()