#!/usr/bin/env python3
"""
ODrive CAN Bridge Node
Reads ODrive motor positions from CAN bus and publishes left/right side positions
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import can
import struct
import math

class OdriveCanInfo(Node):
    def __init__(self):
        super().__init__('odrive_can_info')
        
        # CAN configuration
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('left_motor_ids', [0, 2, 4])   # Left side motor node IDs
        self.declare_parameter('right_motor_ids', [1, 3, 5])  # Right side motor node IDs
        self.declare_parameter('gear_ratio',50)

        self.can_interface = self.get_parameter('can_interface').value
        self.left_motor_ids = self.get_parameter('left_motor_ids').value
        self.right_motor_ids = self.get_parameter('right_motor_ids').value
        self.gear_ratio = self.get_parameter('gear_ratio').value

        self.get_logger().info(f'CAN interface: {self.can_interface}')
        self.get_logger().info(f'Left motor IDs: {self.left_motor_ids}')
        self.get_logger().info(f'Right motor IDs: {self.right_motor_ids}')
        
        # Create publishers for left and right side positions
        self.left_pos_pub = self.create_publisher(Float32, '/odrive/left/position', 10)
        self.right_pos_pub = self.create_publisher(Float32, '/odrive/right/position', 10)
        
        # Track individual motor positions for averaging
        self.left_positions = {motor_id: 0.0 for motor_id in self.left_motor_ids}
        self.right_positions = {motor_id: 0.0 for motor_id in self.right_motor_ids}
        
        # Initialize CAN bus
        try:
            self.bus = can.interface.Bus(self.can_interface, interface="socketcan")
            self.get_logger().info(f'CAN bus initialized on {self.can_interface}')
            
            # Flush CAN RX buffer so there are no more old pending messages
            while not (self.bus.recv(timeout=0) is None):
                pass
                
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CAN bus: {e}')
            raise
        
        # Start CAN processing timer
        self.create_timer(0.001, self.process_can_messages)  # 1000 Hz
        
    def round_value(self, value, sigfig):
        """Round value to significant figures"""
        value = value * (10 ** sigfig)
        if not math.isnan(value):
            value = int(value)
        value = value / (10 ** sigfig)
        return value
    
    def process_can_messages(self):
        """Process CAN messages non-blocking"""
        # Process multiple messages per timer callback
        for _ in range(10):
            msg = self.bus.recv(timeout=0)
            if msg is None:
                break
            
            try:
                # Check left side motors
                for node_id in self.left_motor_ids:
                    if msg.arbitration_id == (node_id << 5 | 0x09):
                        pos_estimate, vel_estimate = struct.unpack('<ff', bytes(msg.data))
                        self.left_positions[node_id] = self.round_value(pos_estimate, 3) * -1 #Position is negative since base is flipped
                        
                        # Publish averaged left position
                        avg_left = sum(self.left_positions.values()) / len(self.left_positions)
                        left_msg = Float32()
                        left_msg.data = avg_left / self.gear_ratio
                        self.left_pos_pub.publish(left_msg)
                
                # Check right side motors
                for node_id in self.right_motor_ids:
                    if msg.arbitration_id == (node_id << 5 | 0x09):
                        pos_estimate, vel_estimate = struct.unpack('<ff', bytes(msg.data))
                        self.right_positions[node_id] = self.round_value(pos_estimate, 3)
                        
                        # Publish averaged right position
                        avg_right = sum(self.right_positions.values()) / len(self.right_positions)
                        right_msg = Float32()
                        right_msg.data = avg_right / self.gear_ratio
                        self.right_pos_pub.publish(right_msg)
                        
            except Exception as e:
                self.get_logger().error(f'Error processing CAN message: {e}')
    
    def destroy_node(self):
        if hasattr(self, 'bus'):
            self.bus.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdriveCanInfo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
