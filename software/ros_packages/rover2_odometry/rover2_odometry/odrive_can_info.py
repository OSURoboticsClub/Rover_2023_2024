#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import can
import struct
import threading
import time


class OdriveCanInfo(Node):
    def __init__(self):
        super().__init__('odrive_can_info')
        
        # CAN configuration
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('left_motor_ids', [0, 2, 4])   # Left side motor node IDs
        self.declare_parameter('right_motor_ids', [1, 3, 5])  # Right side motor node IDs
        self.declare_parameter('gear_ratio', 50)
        self.declare_parameter('data_timeout', 0.2)  # CAN data timeout in seconds

        self.can_interface = self.get_parameter('can_interface').value
        self.left_motor_ids = self.get_parameter('left_motor_ids').value
        self.right_motor_ids = self.get_parameter('right_motor_ids').value
        self.motor_ids = set(self.left_motor_ids + self.right_motor_ids)
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.timeout = self.get_parameter('data_timeout').value

        self.get_logger().info(f'CAN interface: {self.can_interface}')
        self.get_logger().info(f'Left motor IDs: {self.left_motor_ids}')
        self.get_logger().info(f'Right motor IDs: {self.right_motor_ids}')
        
        self.to_radians = 2 * math.pi / self.gear_ratio

        # Initialize CAN bus
        try:
            self.bus = can.interface.Bus(self.can_interface, interface="socketcan")
            self.get_logger().info(f'CAN bus initialized on {self.can_interface}')
            # Flush CAN RX buffer so there are no more old pending messages
            while not (self.bus.recv(timeout=0) is None): pass
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CAN bus: {e}')
            raise
        
        # Create publishers for left and right side
        self.odrive_motor_state_pub = self.create_publisher(JointState, '/odrive/motor_states', 10)

        # Start CAN receive thread
        self.data_lock = threading.Lock()
        self.motor_states = {} # {node_id: {pos, vel, time}}
        self.running = True
        self.recv_thread = threading.Thread(target=self.can_bus_receiver, daemon=True)
        self.recv_thread.start()

        # Start CAN processing timer
        self.create_timer(0.02, self.publish_motor_states)  # 50 Hz
        

    def can_bus_receiver(self):
        self.get_logger().info('CAN receiver thread started')
        while self.running and rclpy.ok():
            try:
                msg = self.bus.recv(timeout=1)
                if msg is None:
                    continue

                node_id = msg.arbitration_id >> 5
                cmd_id = msg.arbitration_id & 0x1F

                if cmd_id == 0x09 and node_id in self.motor_ids:
                    pos, vel = struct.unpack('<ff', bytes(msg.data))
                    with self.data_lock:
                        self.motor_states[node_id] = { 'pos': pos, 'vel': vel, 'time': time.time() }

            except can.CanError as e:
                self.get_logger().error(f'CAN error: {e}')
            except Exception as e:
                self.get_logger().error(f'Error receiving CAN message: {e}')


    def compute_averages(self, ids, snapshot, now):
        pos = 0.0
        vel = 0.0
        count = 0

        for node_id in ids:
            if node_id in snapshot:
                state = snapshot[node_id]
                if (now - state['time']) <= self.timeout:
                    pos += state['pos']
                    vel += state['vel']
                    count += 1

        if count == 0:
            return None, None
        return pos / count, vel / count


    def publish_motor_states(self):

        sys_now = time.time()

        with self.data_lock:
            snapshot = self.motor_states.copy()

        left_pos, left_vel = self.compute_averages(self.left_motor_ids, snapshot, sys_now)
        right_pos, right_vel = self.compute_averages(self.right_motor_ids, snapshot, sys_now)

        if left_pos is None or right_pos is None:
            self.get_logger().warn('No valid motor data available to publish', throttle_duration_sec=1.0)
            return

        left_pos = left_pos * self.to_radians * -1
        left_vel = left_vel * self.to_radians * -1 # Invert left side

        right_pos = right_pos * self.to_radians
        right_vel = right_vel * self.to_radians
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel', 'right_wheel']
        msg.position = [left_pos, right_pos]
        msg.velocity = [left_vel, right_vel]
        self.odrive_motor_state_pub.publish(msg)


    def destroy_node(self):
        self.running = False
        if hasattr(self, 'bus'):
            self.bus.shutdown()
        if hasattr(self, 'recv_thread') and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=2.0)
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
