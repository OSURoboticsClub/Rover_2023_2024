#!/usr/bin/env python3
#####################################
# Imports
#####################################
# Python native imports
from time import time, sleep
import math
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf_transformations import euler_from_quaternion
#####################################
# Global Variables
#####################################
NODE_NAME = "imu"

DEFAULT_IMU_TOPIC = "imu/data"
DEFAULT_MAG_TOPIC = "imu/mag"
IMU_HEADING_TOPIC = "imu/heading"

#####################################
# IMU Node Class Definition
#####################################
class IMUNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        self.imu_data_topic = self.declare_parameter("imu_data_topic", DEFAULT_IMU_TOPIC).value
        self.mag_topic = self.declare_parameter("mag_topic", DEFAULT_MAG_TOPIC).value
        self.imu_heading_topic = self.declare_parameter("imu_heading_topic", IMU_HEADING_TOPIC).value

        # Publishers
        self.imu_data_publisher = self.create_publisher(Imu, self.imu_data_topic, 10)
        self.mag_publisher = self.create_publisher(MagneticField, self.mag_topic, 10)
        self.imu_heading_publisher = self.create_publisher(Float32, self.imu_heading_topic, 10)

        #Subscribers
        self.raw_imu_sub = self.create_subscription(Imu, 'imu/data/raw', self.add_covariance, 10)
        self.raw_mag_sub = self.create_subscription(MagneticField, 'imu/mag/raw', self.add_mag_covariance, 10)
        

    def add_covariance(self, msg):
        imu_msg = msg
        imu_msg.orientation_covariance[0] = 0.01
        imu_msg.orientation_covariance[4] = 0.01
        imu_msg.orientation_covariance[8] = 0.01
        imu_msg.angular_velocity_covariance[0] = 0.001
        imu_msg.angular_velocity_covariance[4] = 0.001
        imu_msg.angular_velocity_covariance[8] = 0.001
        imu_msg.linear_acceleration_covariance[0] = 0.01
        imu_msg.linear_acceleration_covariance[4] = 0.01
        imu_msg.linear_acceleration_covariance[8] = 0.01
        self.imu_data_publisher.publish(imu_msg)
        self.publish_heading(imu_msg)

    def add_mag_covariance(self, msg):
        mag_msg = msg
        mag_msg.magnetic_field_covariance[0] = 0.01
        mag_msg.magnetic_field_covariance[4] = 0.01
        mag_msg.magnetic_field_covariance[8] = 0.01
        
        self.mag_publisher.publish(mag_msg)


    def publish_heading(self, imu_msg):
        """Compute and publish heading from a ROS2 Imu message"""
        try:
            q = imu_msg.orientation

            # Convert quaternion to Euler angles
            quaternion = [q.x, q.y, q.z, q.w]
            roll, pitch, yaw = euler_from_quaternion(quaternion)

            # Convert yaw to degrees
            heading_deg = math.degrees(yaw)

            # Apply magnetic declination (remove this for Gazebo if desired)
            offset_heading = heading_deg #+ self.magnetic_declination

            # Normalize to 0–360
            offset_heading = offset_heading % 360.0

            # Convert to -180 to 180
            if offset_heading > 180:
                offset_heading -= 360

            heading_msg = Float32()
            heading_msg.data = float(offset_heading)

            self.imu_heading_publisher.publish(heading_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing heading: {e}')


def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
