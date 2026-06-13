from rover2_status_interface.msg import LED


class LightsController:
    """
    Helper class for controlling autonomous LEDs.
    """

    TOPIC_NAME = "/autonomous_LED/color"

    def __init__(self, node):
        self.node = node

        self.publisher = self.node.create_publisher(
            LED,
            self.TOPIC_NAME,
            10
        )

    def _publish_color(self, red=0, green=0, blue=0):
        msg = LED()
        msg.red = red
        msg.green = green
        msg.blue = blue
        self.publisher.publish(msg)

    # =========================
    # Light Modes
    # =========================

    def off(self):
        self._publish_color(0, 0, 0)

    def auton_on(self):
        # red
        self._publish_color(1, 0, 0)

    def success(self):
        # green
        self._publish_color(0, 1, 0)

    def blue(self):
        # teleop
        self._publish_color(0, 0, 1)