import rclpy
from rclpy.node import Node
from rover2_status_interface.msg import LED

import time
import serial
import numpy as np

# To Test
# ros2 topic pub /autonomous_LED/color rover2_status_interface/msg/LED "{red: 0, green: 0, blue: 1}"

NODE_NAME = "autonomous_LED"

# Serial color commands
OFF = 0
RED = 1
GREEN = 2
BLUE = 3
COLORFUL = 4

class AutonomousLEDSubscriber(Node):

    def __init__(self):
        super().__init__(NODE_NAME)

        self.arduino = serial.Serial(port="/dev/rover/lights",baudrate=9600, timeout=0.1)
        time.sleep(2)
        self.arduino.write(str(BLUE).encode('utf-8'))
        self.subscription = self.create_subscription(
            LED,
            '/autonomous_LED/color',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        command = np.argmax([0, msg.red, msg.green, msg.blue])
        self.arduino.write(str(command).encode('utf-8'))
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
