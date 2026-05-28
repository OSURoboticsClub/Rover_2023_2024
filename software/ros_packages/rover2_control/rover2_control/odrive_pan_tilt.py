import rclpy
from rclpy.node import Node

#Can imports
import can 
import struct

from time import time, sleep


from rover2_control_interface.msg import OdrivePanTiltControlMessage

class OdrivePanTilt(Node):

    def __init__(self):
        super().__init__('odrive_pan_tilt')
        #parameters
        self.declare_parameter('control_topic', "/chasis_pan_tilt/control")
        self.declare_parameter('can', "can0")
        self.declare_parameter('direction', 1)
        self.declare_parameter('node_ids', [0, 1, 2])
        self.declare_parameter('board_id', 0)



        self.control_topic = self.get_parameter('control_topic').value
        self.can_network = self.get_parameter('can').value
        self.direction = self.get_parameter('direction').value
        self.node_ids = self.get_parameter('node_ids').value
        self.board_id = self.get_parameter('board_id').value

        self.control_sub = self.create_subscription(
            OdrivePanTiltControlMessage,
            self.control_topic,
            self.control_callback,
            10
        )

        self.V_LIMIT = 3.1415 #[rad/s]
        self.PUBLISH_RATE = 0.03 #[s]
        self.dt = self.V_LIMIT * self.PUBLISH_RATE #[rad] per publish

        self.go_home = True
        self.is_angle = False
        self.angles = [0.0, 0.0, 0.0] # pitch, yaw, roll
        self.stabalize = False
        self.last_stable = False

        #Can setup
        self.bus = can.interface.Bus(channel=self.can_network, bustype='socketcan')
        # self.can_timer = self.create_timer(0.005, self.read_can)
        # while not (self.bus.recv(timeout=0) is None): pass

        self.timer = self.create_timer(0.03, self.timer_callback)

    def timer_callback(self):
        self.send_angles()
        if self.stabalize != self.last_stable:
            self.send_stable()

    
    def send_stable(self):
        try:
            self.bus.send(can.Message(
                arbitration_id = (self.board_id << 5 | 0x02), #toggle Stabalization
                data = struct.pack('<?', self.stabalize),
                is_extended_id = False
            ))
            self.last_stable = self.stabalize
            self.get_logger().info(f"CAN: Sent Can for angles: roll={self.angles[0]}, pitch={self.angles[1]}, yaw={self.angles[2]}")
        except:
            self.get_logger().info("CAN: CAN Buffer full")

    def send_angles(self):
        try:
            for i, node_id in enumerate(self.node_ids):
                if node_id < 0:
                    continue
                self.bus.send(can.Message(
                    arbitration_id = (self.board_id << 5 | 0x01), #Motor control
                    data = struct.pack('<Bf', node_id, self.angles[i]),
                    is_extended_id = False
                ))
            self.get_logger().info(f"CAN: Sent Can for angles: roll={self.angles[0]}, pitch={self.angles[1]}, yaw={self.angles[2]}")
        except:
            self.get_logger().info("CAN: CAN Buffer full")

    def control_callback(self, msg):
        self.get_logger().info(f"CONTROL: Recieved control message: {msg}")
        if msg.go_home:
            self.get_logger().info("CONTROL: Going home!")
            self.go_home = True
            self.angles[0] = 0.0
            self.angles[1] = 0.0
            self.angles[2] = 0.0
        elif msg.is_angle:
            self.get_logger().info(f"CONTROL: Recieved angles: roll={msg.roll}, pitch={msg.pitch}, yaw={msg.yaw}")
            self.is_angle = True
            self.angles[0] = msg.pitch * self.direction
            self.angles[1] = msg.yaw * self.direction
            self.angles[2] = msg.roll * self.direction
        else:
            self.is_angle = False
            self.angles[0] = self.angles[0] + msg.pitch * self.dt * self.direction
            self.angles[1] = self.angles[1] + msg.yaw * self.dt * self.direction
            self.angles[2] = self.angles[2] + msg.roll * self.dt * self.direction
            self.get_logger().info(f"CONTROL: Stepped Angles positions: roll={self.angles[0]}, pitch={self.angles[1]}, yaw={self.angles[2]}")
        if msg.stabalize:
            self.get_logger().info("CONTROL: Stabilize enabled")
            self.stabalize = msg.stabalize

def main(args=None):
    rclpy.init(args=args)

    odrive_pan_tilt = OdrivePanTilt()

    rclpy.spin(odrive_pan_tilt)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odrive_pan_tilt.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
