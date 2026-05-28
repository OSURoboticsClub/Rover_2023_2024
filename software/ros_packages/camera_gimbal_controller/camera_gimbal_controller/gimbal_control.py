import camera_gimbal_controller.console_ui as c
import camera_gimbal_controller.console_ui_elements as ce
import rclpy
import math
from rclpy.node import Node
from rover2_control_interface.msg import OdrivePanTiltControlMessage


console = c.ConsoleUI(exit, automaticSize=False, sizeX=20, sizeY=10)

titleFg = c.Color(255,255,255)
primaryFg = c.Color(200,200,200)
primaryBg = c.Color(0,0,128)

title = ce.StringField(console, 0, 0, "Gimbal Controller", titleFg, primaryBg)

angles = ce.TableField(console, 1, 2, dict(), "Angles", fg=primaryFg, bg=primaryBg, title_fg=titleFg, title_bg=primaryBg)


class GimbalControlNode(Node):

    def changeAngles(self, p = 0.0, r=0.0, y=0.0):
        self.pitch = p
        self.roll = r
        self.yaw = y

        msg = OdrivePanTiltControlMessage()

        msg.go_home = False
        msg.stabalize = False
        msg.is_angle = True

        msg.pitch = math.radians(self.pitch)
        msg.roll = math.radians(self.roll)
        msg.yaw = math.radians(self.yaw)

        self.control_publisher.publish(msg)


    def home(self):

        msg = OdrivePanTiltControlMessage()

        msg.go_home = True
        msg.stabalize = False
        msg.is_angle = False

        self.control_publisher.publish(msg)
        

    def __init__(self):
        super().__init__("gimbal_controller")

        self.step = self.declare_parameter("step_size", 1.0).value
        self.target_topic = self.declare_parameter("gimbal_topic", "/gimbal").value

        self.control_publisher = self.create_publisher(
            OdrivePanTiltControlMessage,
            self.target_topic if self.target_topic is not None else "/gimbal",
            10
        )

        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0

        console.bind_key(ord('w'), lambda: self.changeAngles(self.pitch + 1, self.roll, self.yaw))
        console.bind_key(ord('a'), lambda: self.changeAngles(self.pitch, self.roll, self.yaw + 1))
        console.bind_key(ord('s'), lambda: self.changeAngles(self.pitch - 1, self.roll, self.yaw))
        console.bind_key(ord('d'), lambda: self.changeAngles(self.pitch, self.roll, self.yaw - 1))
        console.bind_key(ord('q'), lambda: self.changeAngles(self.pitch, self.roll + 1, self.yaw))
        console.bind_key(ord('e'), lambda: self.changeAngles(self.pitch, self.roll - 1, self.yaw))

        console.bind_key(ord("h"), lambda: self.home())


def main(args=None):
    rclpy.init()

    node = GimbalControlNode()

    while(True):
        rclpy.spin_once(node, timeout_sec=.05)
        angles.update_row("Pitch:", str(node.pitch))
        angles.update_row("Roll:", str(node.roll))
        angles.update_row("Yaw:", str(node.yaw))
        console.update(bg=primaryBg)


