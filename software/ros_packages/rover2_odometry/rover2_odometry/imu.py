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
from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
#####################################
# Global Variables
#####################################
NODE_NAME = "imu"

DEFAULT_IMU_TOPIC = "imu/data"
IMU_HEADING_TOPIC = "imu/data/heading"

DEFAULT_HERTZ = 1000

#####################################
# Odometry Class Definition
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


class Odometry(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.imu_data_topic = self.declare_parameter("~imu_data_topic", DEFAULT_IMU_TOPIC).value
        self.imu_heading_topic = self.declare_parameter("~imu_heading_topic", IMU_HEADING_TOPIC).value
        self.wait_time = 1.0 / self.declare_parameter('~hertz', DEFAULT_HERTZ).value

        self.imu_data_publisher = self.create_publisher(Imu, self.imu_data_topic, 1)
        self.imu_heading_publisher = self.create_publisher(Float32, self.imu_heading_topic,1)
        print(self.wait_time)
        self.timer = self.create_timer(self.wait_time, self.main_loop)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = adafruit_bno055.BNO055_I2C(self.i2c)
        self.initial_offset = self.get_initial_offset()
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
	    'mag_radius': 732, # DO NOT CHANGE
	    'accel_radius': 1000 # DO NOT CHANGE
        }
        self.imu.mode = Mode.CONFIG_MODE
        self.set_calibration_offsets()
        self.write_calibration_offsets()

#        self.imu.mode = Mode.NDOF_FMC_OFF_MODE
        self.imu.mode = Mode.NDOF_MODE
    def write_offset(self,register, value):
        # Convert value to little-endian format
        value_bytes = value.to_bytes(2, 'little', signed=True)
        self.imu._write_register(register, value_bytes[0])  # Write lower byte
        self.imu._write_register(register + 1, value_bytes[1])  # Write upper byte


    def main_loop(self):
        try:
            self.process_messages()

        except Exception as error:
            print(error)
            pass

    def set_calibration_offsets(self):
        
        with open('/home/makemorerobot/Rover_2023_2024/software/ros_packages/rover2_odometry/rover2_odometry/calibration_settings.txt', 'r') as file:
            line = file.readline()
            offsets = line.split(" ")
            i = 0
            for key in self.calibration_offsets:
                if i > 8:
                    return
                self.calibration_offsets[key] = int(offsets[i])
                i+=1
    def write_calibration_offsets(self):
        self.write_offset(0x55, self.calibration_offsets['acc_offset_x'])  # ACC_OFFSET_X_LSB
        self.write_offset(0x57, self.calibration_offsets['acc_offset_y'])  # ACC_OFFSET_Y_LSB
        self.write_offset(0x59, self.calibration_offsets['acc_offset_z'])  # ACC_OFFSET_Z_LSB

        # Write magnetometer offsets
        self.write_offset(0x5B, self.calibration_offsets['mag_offset_x'])  # MAG_OFFSET_X_LSB
        self.write_offset(0x5D, self.calibration_offsets['mag_offset_y'])  # MAG_OFFSET_Y_LSB
        self.write_offset(0x5F, self.calibration_offsets['mag_offset_z'])  # MAG_OFFSET_Z_LSB

        # Write gyroscope offsets
        self.write_offset(0x61, self.calibration_offsets['gyr_offset_x'])  # GYR_OFFSET_X_LSB
        self.write_offset(0x63, self.calibration_offsets['gyr_offset_y'])  # GYR_OFFSET_Y_LSB
        self.write_offset(0x65, self.calibration_offsets['gyr_offset_z'])  # GYR_OFFSET_Z_LSB

        # Write magnetometer and accelerometer radius 
        self.write_offset(0x67, self.calibration_offsets['accel_radius'])  #ACC_RADIUS_LSB
        self.write_offset(0x69, self.calibration_offsets['mag_radius'])  #ACC_RADIUS_LSB
        print("Sensor offsets set successfully!")
        print(self.calibration_offsets)

    def get_initial_offset(self):
        euler_angles = self.imu.euler
        if euler_angles:
            heading = euler_angles[0]  # Heading is the first element (in degrees)
            return euler_angles[0]
        else:
            print("Sensor data unavailable.")
        return 0

        
    def process_messages(self):
        self.broadcast_imu()
    """
    def quat_to_euler(self):
        r = R.from_quat([self.imu.quaternion[2], self.imu.quaternion[1], self.imu.quaternion[0], self.imu.quaternion[3]])  # Note: scipy uses (x, y, z, w)
        euler_angles = r.as_euler('xyz', degrees=True)  # 'xyz' represents the order of rotation axes

        # Convert to degrees (optional)
        euler_angles_degrees = r.as_euler('xyz', degrees=True)

        #print("Euler angles (radians):", euler_angles)
        #print("Euler angles (degrees):", euler_angles_degrees)
    """

    def broadcast_imu(self):
        #self.quat_to_euler()
        """
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
        """
        heading_message = Float32()
        offset_heading = self.imu.euler[0] # - self.initial_offset
#        if(offset_heading > 180):
#            offset_heading-=360
        heading_message.data = offset_heading
        self.imu_heading_publisher.publish(heading_message)
def main(args=None):
        rclpy.init(args=args)
        odometry = Odometry()
        rclpy.spin(odometry)
        odometry.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
