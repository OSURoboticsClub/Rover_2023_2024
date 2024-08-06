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

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
import threading
import cv2
import numpy as np
from flask import Flask, send_file, request
from cv_bridge import CvBridge
import asyncio
import websocket
from socketIO_client import SocketIO, LoggingNamespace
import sys
import json
import requests
import base64
import time
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

IMG_BYTES = b""
app = Flask("img_backend")

@app.route("/img")
def image():
    return send_file(IMG_BYTES, mimetype='image/jpeg')

url1 = "http://localhost:5000/img1"
headers = {'Content-type': 'application/json', 'Accept': 'text/plain'}
class MinimalSubscriber(Node):

    def __init__(self):
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        
        )
        super().__init__('minimal_subscriber')
        self.bridge = CvBridge()
        self.img_bytes = b""
        
        self.subscription = self.create_subscription(
            CompressedImage,
            '/cameras/main_navigation/image_256x144/compressed',
            self.tower_listener_callback,
             qos_profile)
        
        self.subscription = self.create_subscription(
            CompressedImage,
            '/cameras/chassis/image_256x144/compressed',
            self.chassis_listener_callback,
             qos_profile)
        
        self.subscription = self.create_subscription(
            CompressedImage,
            '/cameras/infrared/image_256x144/compressed',
            self.infrared_listener_callback,
             qos_profile)
        
        self.subscription = self.create_subscription(
             CompressedImage,
             '/cameras/gripper/image_256x144/compressed',
             self.gripper_listener_callback,
             qos_profile)
    
        # self.publisher = self.create_publisher(CompressedImage, '/cameras/main_navigation/forward', 1)
        self.subscription  # prevent unused variable warning

    def tower_listener_callback(self, msg):
        
        opencv_image = cv2.resize(self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8"), (640, 360))
        cv2.imshow('tower', opencv_image)
        cv2.waitKey(1)

    def chassis_listener_callback(self, msg):
        opencv_image = cv2.resize(self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8"), (640, 360))
        cv2.imshow('chassis', opencv_image)
        cv2.waitKey(1)

    def infrared_listener_callback(self, msg):
        opencv_image = cv2.resize(self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8"), (640, 360))
        cv2.imshow('infrared', opencv_image)
        cv2.waitKey(1)

    def gripper_listener_callback(self, msg):
        opencv_image = cv2.resize(self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8"), (640, 360))
        cv2.imshow('gripper', opencv_image)
        cv2.waitKey(1)
        # _, img = cv2.imencode('.jpg', opencv_image)
        # IMG_BYTES = img.tobytes()
        # self.publisher.publish(msg)

        # img_json = json.dumps({"msg": base64.b64encode(img_bytes).decode()})
        # #print(img_json[:100])
        # #time.sleep(100)
        # try:
        #     r1 = requests.post(url1, data=img_json, headers=headers,timeout = 0.03)
        # except requests.exceptions.ReadTimeout: 
        #     pass
        # except requests.exceptions.ConnectionError:
        #     print("Backend server is offline, please check it's status. Exiting...")
        
        # #data = {'msg': 'Hi!!!'}
        # #headers = {'Content-type': 'application/json', 'Accept': 'text/plain'}
        # #r = requests.post(self.url, data=json.dumps(data), headers=headers)
        



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
