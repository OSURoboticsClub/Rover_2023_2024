import rclpy
import rclpy.logging
from rclpy.node import Node
import serial
from std_msgs.msg import Float32MultiArray
from time import time, sleep

class ScimechSensors(Node):

    def __init__(self):

        super().__init__('scimech_sensors')

        self.scimech_data_publisher_ = self.create_publisher(Float32MultiArray, 'scimech/data', 10)
        
        timer_period = 0.033  # seconds

        self.logger = self.get_logger()

        self.humidity = 0.0
        self.hydrogen = 0.0
        self.ozone = 0.0
        self.temperature = 0.0

        self.scimech_arduino = serial.Serial('/dev/ttyCH341USB0',115200, timeout=1)
        self.scimech_arduino.flush()
        self.timer = self.create_timer(timer_period, self.timer_callback)        
  
        self.hydrogen_index = 0
        self.ozone_index = 1
        self.geiger_index = 2

    def read_scimech_serial_data(self):
        while True:
            if self.scimech_arduino.in_waiting > 0:
                try: 
                    line = self.scimech_arduino.readline().decode('utf-8').rstrip()
                    if line and line!="ACKSERVO":
                        self.logger.info("Reading from arduino: {}".format(line))
                        data_array = line.split(",")
                        if(len(data_array)>1):
                        
                            self.hydrogen = float(data_array[self.hydrogen_index])
                            self.ozone = float(data_array[self.ozone_index])
                            self.geiger = float(data_array[self.geiger_index])
                        self.scimech_arduino.flush()
                    
                
                except Exception as e:
                    self.logger.info(e)
                break

    def timer_callback(self):
        
        self.read_scimech_serial_data()
        msg = Float32MultiArray()
        msg.data = [self.hydrogen,self.ozone,self.geiger]
        self.scimech_data_publisher_.publish(msg)
        self.logger.info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    scimech_sensors = ScimechSensors()

    rclpy.spin(scimech_sensors)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scimech_sensors.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
