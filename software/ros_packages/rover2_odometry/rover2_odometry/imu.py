#!/usr/bin/env python3
#####################################
# Imports
#####################################
# Python native imports
from time import time, sleep
import board
import busio
import adafruit_bno055
import math
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node

#####################################
# Global Variables
#####################################
NODE_NAME = "imu"

DEFAULT_IMU_TOPIC = "imu/data"
DEFAULT_MAG_TOPIC = "imu/mag"
IMU_HEADING_TOPIC = "imu/heading"

DEFAULT_HERTZ = 100  # Changed from 1000 to reasonable rate

#####################################
# Mode Class Definition
#####################################
class Mode:
    CONFIG_MODE = 0x00
    ACCONLY_MODE = 0x01
    MAGONLY_MODE = 0x02
    GYRONLY_MODE = 0x03
    ACCMAG_MODE = 0x04
    ACCGYRO_MODE = 0x05
    MAGGYRO_MODE = 0x06
    AMG_MODE = 0x07
    IMUPLUS_MODE = 0x08
    COMPASS_MODE = 0x09
    M4G_MODE = 0x0A
    NDOF_FMC_OFF_MODE = 0x0B
    NDOF_MODE = 0x0C


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
        self.wait_time = 1.0 / self.declare_parameter('hertz', DEFAULT_HERTZ).value

        # Publishers
        self.imu_data_publisher = self.create_publisher(Imu, self.imu_data_topic, 10)
        self.mag_publisher = self.create_publisher(MagneticField, self.mag_topic, 10)
        self.imu_heading_publisher = self.create_publisher(Float32, self.imu_heading_topic, 10)
        
        self.get_logger().info(f'Publishing at {1.0/self.wait_time} Hz')
        
        # Initialize I2C and BNO055
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = adafruit_bno055.BNO055_I2C(self.i2c)
        
        # Magnetic declination (adjust for your location)
        self.magnetic_declination = 0
        
        # Calibration offsets
        self.calibration_offsets = {
            'acc_offset_x': -37,
            'acc_offset_y': 7,
            'acc_offset_z': -30,
            'mag_offset_x': -257,
            'mag_offset_y': -334,
            'mag_offset_z': -584,
            'gyr_offset_x': 1,
            'gyr_offset_y': -5,
            'gyr_offset_z': -3,
            'mag_radius': 732,
            'accel_radius': 1000
        }
        
        # Configure sensor
        self.imu.mode = Mode.CONFIG_MODE
        self.set_calibration_offsets()
        self.set_magnetic_declination()
        self.write_calibration_offsets()
        
        # Set to NDOF mode for full 9-DOF fusion
        self.imu.mode = Mode.NDOF_MODE
        self.get_logger().info('BNO055 configured in NDOF mode (full 9-DOF fusion)')
        
        # Start timer
        self.timer = self.create_timer(self.wait_time, self.main_loop)

    def write_offset(self, register, value):
        """Write calibration offset to sensor register"""
        value_bytes = value.to_bytes(2, 'little', signed=True)
        self.imu._write_register(register, value_bytes[0])
        self.imu._write_register(register + 1, value_bytes[1])

    def set_calibration_offsets(self):
        """Load calibration offsets from file"""
        try:
            with open('/home/makemorerobot/Rover_2023_2024/software/ros_packages/rover2_odometry/rover2_odometry/calibration_settings.txt', 'r') as file:
                line = file.readline()
                offsets = line.split(" ")
                i = 0
                for key in self.calibration_offsets:
                    if i > 8:
                        break
                    self.calibration_offsets[key] = int(offsets[i])
                    i += 1
            self.get_logger().info('Loaded calibration offsets from file')
        except Exception as e:
            self.get_logger().warn(f'Could not load calibration file: {e}')

    def set_magnetic_declination(self):
        """Load magnetic declination from file"""
        try:
            with open('/home/makemorerobot/Rover_2023_2024/software/ros_packages/rover2_odometry/rover2_odometry/magnetic_declination.txt', 'r') as file:
                line = file.readline()
                self.magnetic_declination = float(line)
            self.get_logger().info(f'Magnetic declination set to {self.magnetic_declination}°')
        except Exception as e:
            self.get_logger().warn(f'Could not load magnetic declination: {e}')

    def write_calibration_offsets(self):
        """Write all calibration offsets to sensor"""
        # Accelerometer offsets
        self.write_offset(0x55, self.calibration_offsets['acc_offset_x'])
        self.write_offset(0x57, self.calibration_offsets['acc_offset_y'])
        self.write_offset(0x59, self.calibration_offsets['acc_offset_z'])

        # Magnetometer offsets
        self.write_offset(0x5B, self.calibration_offsets['mag_offset_x'])
        self.write_offset(0x5D, self.calibration_offsets['mag_offset_y'])
        self.write_offset(0x5F, self.calibration_offsets['mag_offset_z'])

        # Gyroscope offsets
        self.write_offset(0x61, self.calibration_offsets['gyr_offset_x'])
        self.write_offset(0x63, self.calibration_offsets['gyr_offset_y'])
        self.write_offset(0x65, self.calibration_offsets['gyr_offset_z'])

        # Radius values
        self.write_offset(0x67, self.calibration_offsets['accel_radius'])
        self.write_offset(0x69, self.calibration_offsets['mag_radius'])
        
        self.get_logger().info('Sensor offsets written successfully')

    def main_loop(self):
        """Main processing loop"""
        try:
            self.publish_imu_data()
            self.publish_heading()
        except Exception as error:
            self.get_logger().error(f'Error in main loop: {error}')

    def publish_imu_data(self):
        try:
            # Get current time
            current_time = self.get_clock().now().to_msg()
            
            # === Publish standard IMU message ===
            imu_msg = Imu()
            imu_msg.header.stamp = current_time
            imu_msg.header.frame_id = "imu_link"
            
            # Orientation (quaternion from BNO055 sensor fusion)
            quat = self.imu.quaternion
            if quat and None not in quat:
                # BNO055 returns (w, x, y, z), ROS expects (x, y, z, w)
                imu_msg.orientation.x = float(quat[1])
                imu_msg.orientation.y = float(quat[2])
                imu_msg.orientation.z = float(quat[3])
                imu_msg.orientation.w = float(quat[0])
                
                # Orientation covariance (adjust based on your sensor accuracy)
                imu_msg.orientation_covariance[0] = 0.01
                imu_msg.orientation_covariance[4] = 0.01
                imu_msg.orientation_covariance[8] = 0.01
            else:
                # If quaternion not available, mark as invalid
                imu_msg.orientation_covariance[0] = -1
            
            # Angular velocity (gyroscope) in rad/s
            gyro = self.imu.gyro
            if gyro and None not in gyro:
                # Convert from deg/s to rad/s
                imu_msg.angular_velocity.x = math.radians(float(gyro[0]))
                imu_msg.angular_velocity.y = math.radians(float(gyro[1]))
                imu_msg.angular_velocity.z = math.radians(float(gyro[2]))
                
                # Angular velocity covariance
                imu_msg.angular_velocity_covariance[0] = 0.001
                imu_msg.angular_velocity_covariance[4] = 0.001
                imu_msg.angular_velocity_covariance[8] = 0.001
            else:
                imu_msg.angular_velocity_covariance[0] = -1
            
            # Linear acceleration in m/s²
            accel = self.imu.linear_acceleration  # Gravity already removed
            if accel and None not in accel:
                imu_msg.linear_acceleration.x = float(accel[0])
                imu_msg.linear_acceleration.y = float(accel[1])
                imu_msg.linear_acceleration.z = float(accel[2])
                
                # Linear acceleration covariance
                imu_msg.linear_acceleration_covariance[0] = 0.01
                imu_msg.linear_acceleration_covariance[4] = 0.01
                imu_msg.linear_acceleration_covariance[8] = 0.01
            else:
                imu_msg.linear_acceleration_covariance[0] = -1
            
            self.imu_data_publisher.publish(imu_msg)
            
            # === Publish magnetometer data separately ===
            mag = self.imu.magnetic
            if mag and None not in mag:
                mag_msg = MagneticField()
                mag_msg.header.stamp = current_time
                mag_msg.header.frame_id = "imu_link"
                
                # Magnetometer data in micro-Tesla (µT)
                mag_msg.magnetic_field.x = float(mag[0])
                mag_msg.magnetic_field.y = float(mag[1])
                mag_msg.magnetic_field.z = float(mag[2])
                
                # Magnetometer covariance
                mag_msg.magnetic_field_covariance[0] = 0.01
                mag_msg.magnetic_field_covariance[4] = 0.01
                mag_msg.magnetic_field_covariance[8] = 0.01
                
                self.mag_publisher.publish(mag_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing IMU data: {e}')

    def publish_heading(self):
        """Publish compass heading in degrees"""
        try:
            euler = self.imu.euler
            if euler and euler[0] is not None:
                # Get heading and apply magnetic declination
                offset_heading = euler[0] + self.magnetic_declination
                
                # Normalize to 0-360
                if offset_heading > 360:
                    offset_heading -= 360
                elif offset_heading < 0:
                    offset_heading += 360
                
                # Convert to -180 to 180 range
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
