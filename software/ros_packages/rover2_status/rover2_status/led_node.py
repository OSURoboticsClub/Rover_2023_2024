import rclpy
from rclpy.node import Node
from rover2_status_interface.msg import LED

import time
import board
import neopixel_spi as neopixel

NODE_NAME = "autonomous_LED"

PIXEL_ORDER = neopixel.RGB
NUM_PIXELS = 11
SPI = board.SPI()
class AutonomousLEDSubscriber(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        self.pixels = neopixel.NeoPixel_SPI(
            SPI, NUM_PIXELS, pixel_order=PIXEL_ORDER, auto_write=False
        )
        for i in range(NUM_PIXELS):
            self.pixels[i] = (0, 0, 255)
        self.pixels.show()

        self.subscription = self.create_subscription(
            LED,
            '/autonomous_LED/color',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        for i in range(NUM_PIXELS):
            self.pixels[i] = (msg.green, msg.red, msg.blue)
        self.pixels.show()
        self.get_logger().info('I heard: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    autonomous_LED_subscriber = AutonomousLEDSubscriber()

    rclpy.spin(autonomous_LED_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    autonomous_LED_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
