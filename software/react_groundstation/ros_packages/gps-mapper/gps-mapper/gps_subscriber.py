# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import csv
import rclpy
from rclpy.node import Node
from rover2_control_interface.msg import GPSStatusMessage

import numpy as np
from flask import Flask, send_file, request
from cv_bridge import CvBridge

import argparse

import cv2
import sys




class MinimalSubscriber(Node):

    def __init__(self):
        self.frames = 0
        
        super().__init__('minimal_subscriber')
        
        
        
        self.subscription = self.create_subscription(
            GPSStatusMessage,
            '/tower/status/gps',
            self.gps_listener_callback,
            1
            )
        
        
    
        # self.publisher = self.create_publisher(CompressedImage, '/cameras/main_navigation/forward', 1)
        self.subscription  # prevent unused variable warning

    

    
    

    def gps_listener_callback(self, msg):
        if(self.frames == 50):
            with open("/home/groundstation/github/Rover_2023_2024/software/react_groundstation/ros_packages/gps-mapper/gps-mapper/coords.csv", "a") as myfile:
                writer = csv.writer(myfile)
                writer.writerow([msg.rover_latitude,msg.rover_longitude])
                print("Writing...")
            self.frames = 0
        
        self.frames+=1
        
        



def main(args=None):
    rclpy.init(args=args)
    

    minimal_subscriber = MinimalSubscriber()

    # app.run()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
