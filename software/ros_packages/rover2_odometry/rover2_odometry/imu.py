#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
from time import time, sleep
import board
import busio
import adafruit_bno055
import math
from sensor_msgs.msg import Imu
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
#####################################
# Global Variables
#####################################
NODE_NAME = "imu"

DEFAULT_IMU_TOPIC = "imu/data"

DEFAULT_HERTZ = 1000



#####################################
# Odometry Class Definition
#####################################
class Odometry(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.imu_data_topic = self.declare_parameter("~imu_data_topic", DEFAULT_IMU_TOPIC).value

        self.wait_time = 1.0 / self.declare_parameter('~hertz', DEFAULT_HERTZ).value

        self.imu_data_publisher = self.create_publisher(Imu, self.imu_data_topic, 1)

        print(self.wait_time)
        self.timer = self.create_timer(self.wait_time, self.main_loop)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = adafruit_bno055.BNO055_I2C(self.i2c)
    def main_loop(self):
        try:
            self.process_messages()

        except Exception as error:
            print(error)
            pass

    def process_messages(self):
        self.broadcast_imu()
    def quat_to_euler(self):
        r = R.from_quat([self.imu.quaternion[2], self.imu.quaternion[1], self.imu.quaternion[0], self.imu.quaternion[3]])  # Note: scipy uses (x, y, z, w)
        euler_angles = r.as_euler('xyz', degrees=True)  # 'xyz' represents the order of rotation axes

        # Convert to degrees (optional)
        euler_angles_degrees = r.as_euler('xyz', degrees=True)

        #print("Euler angles (radians):", euler_angles)
        #print("Euler angles (degrees):", euler_angles_degrees)
    def broadcast_imu(self):
        self.quat_to_euler()
        message = Imu()
        message.header.frame_id = "imu"
        message.header.stamp = self.get_clock().now().to_msg()

        message.orientation.x = self.imu.quaternion[0]
        message.orientation.y = self.imu.quaternion[1]
        message.orientation.z = self.imu.quaternion[2]
        message.orientation.w = self.imu.quaternion[3]

        message.angular_velocity.x = self.imu.gyro[0]
        message.angular_velocity.y = self.imu.gyro[1]
        message.angular_velocity.z = self.imu.gyro[2]

        message.linear_acceleration.x = self.imu.acceleration[0]
        message.linear_acceleration.y = self.imu.acceleration[1]
        message.linear_acceleration.z = self.imu.acceleration[2]
        self.imu_data_publisher.publish(message)

def main(args=None):
        rclpy.init(args=args)
        odometry = Odometry()
        rclpy.spin(odometry)
        odometry.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
