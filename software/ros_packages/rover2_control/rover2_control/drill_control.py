import rclpy
from rclpy.node import Node
import can
import struct
import serial

from std_msgs.msg import Float32
from std_msgs.msg import Int16
RPS_FACTOR = 100
GEAR_RATIO = 55
VEL_RAMP = GEAR_RATIO * RPS_FACTOR

NODE_ID = 6

class DrillControl(Node):

    def __init__(self):
        self.position = 500
        super().__init__('drill_control')
        self.subscription = self.create_subscription(
            Float32,
            'drill/control',
            self.listener_callback,
            10)
        self.compartment_subcription = self.create_subscription(
            Int16,
            'drill/compartment',
            self.compartment_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.arduino = serial.Serial(port="/dev/ttyACM0",baudrate=9600, write_timeout=0)
        self.bus = can.interface.Bus("can1", interface="socketcan")
        # Flush CAN RX buffer so there are no more old pending messages
        while not (self.bus.recv(timeout=0) is None): pass
# Put axis into closed loop control state
        self.bus.send(can.Message(
        arbitration_id=(NODE_ID << 5 | 0x07), # 0x07: Set_Axis_State
        data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
        is_extended_id=False
        ))

    #Set velocity ramp control mode
        self.bus.send(can.Message(
        arbitration_id=(NODE_ID << 5 | 0x0b), 
        data=struct.pack('<II', 2,2),
        is_extended_id=False
        ))
    #Set veloicity ramp speed
        self.bus.send(can.Message(
        arbitration_id=(NODE_ID << 5 | 0x04), 
        data=struct.pack('<BHBf', 1,403,0,VEL_RAMP),
        is_extended_id=False
        ))


#        self.bus.send(can.Message(
#        arbitration_id=(NODE_ID << 5 | 0x0d), # 0x0d: Set_Input_Vel
#        data=struct.pack('<ff', 0.0, 0.0), # 1.0: velocity, 0.0: torque feedforward
#        is_extended_id=False
#        ))
        self.velocity = 0
        self.timer = self.create_timer(0.1, self.timer_callback)


    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.velocity = msg.data * -1 * GEAR_RATIO



    def timer_callback(self):
        self.arduino.write(bytes(str(self.position),'utf-8'))
        self.bus.send(can.Message(
        arbitration_id=(NODE_ID << 5 | 0x0d), # 0x0d: Set_Input_Vel
        data=struct.pack('<ff', self.velocity, 0.0), # 1.0: velocity, 0.0: torque feedforward
        is_extended_id=False
        ))

        
    def compartment_callback(self,msg):
        self.position = msg.data


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = DrillControl()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
