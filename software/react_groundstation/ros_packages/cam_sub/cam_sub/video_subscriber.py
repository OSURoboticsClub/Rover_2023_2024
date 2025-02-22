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
from std_msgs.msg import ByteMultiArray
from sensor_msgs.msg import CompressedImage
import subprocess as sp
import cv2
import numpy as np
import binascii
from cv_bridge import CvBridge
import asyncio
import websocket
import ffmpeg
import sys
import json
import requests
import base64
import time
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

import argparse

import cv2
import sys

IMG_BYTES = b""

ARUCO_DICT = [
	cv2.aruco.DICT_4X4_50,
	 cv2.aruco.DICT_4X4_100,
	 cv2.aruco.DICT_4X4_250,
	cv2.aruco.DICT_4X4_1000,
	cv2.aruco.DICT_5X5_50,
	 cv2.aruco.DICT_5X5_100,
	 cv2.aruco.DICT_5X5_250,
	cv2.aruco.DICT_5X5_1000,
	cv2.aruco.DICT_6X6_50,
	 cv2.aruco.DICT_6X6_100,
	 cv2.aruco.DICT_6X6_250,
	cv2.aruco.DICT_6X6_1000,
	cv2.aruco.DICT_7X7_50,
	 cv2.aruco.DICT_7X7_100,
	 cv2.aruco.DICT_7X7_250,
	cv2.aruco.DICT_7X7_1000,
	 cv2.aruco.DICT_ARUCO_ORIGINAL,
	cv2.aruco.DICT_APRILTAG_16h5,
	cv2.aruco.DICT_APRILTAG_25h9,
	 cv2.aruco.DICT_APRILTAG_36h10,
	 cv2.aruco.DICT_APRILTAG_36h11
]


def image():
    return send_file(IMG_BYTES, mimetype='image/jpeg')

url1 = "http://localhost:5000/img1"
headers = {'Content-type': 'application/json', 'Accept': 'text/plain'}
class MinimalSubscriber(Node):

    def __init__(self):
        self.chassisFrames = 0
        self.towerFrames = 0
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        
        )
        
        super().__init__('minimal_subscriber')
        self.bridge = CvBridge()
        self.img_bytes = b""
        """
        self.subscription1 = self.create_subscription(
            CompressedImage,
            '/cameras/main_navigation/image_256x144/compressed',
            self.tower_listener_callback,
             qos_profile)
        
        self.subscription2 = self.create_subscription(
            CompressedImage,
            '/cameras/chassis/image_256x144',
            self.chassis_listener_callback,
             qos_profile)
        """
        self.subscription3 = self.create_subscription(
            ByteMultiArray,
            '/cameras/infrared/image_800x600',
            self.infrared_listener_callback,
             qos_profile)
        """
        self.subscription4 = self.create_subscription(
             CompressedImage,
             '/cameras/gripper/image_256x144',
             self.gripper_listener_callback,
             qos_profile)
        """
        # self.publisher = self.create_publisher(CompressedImage, '/cameras/main_navigation/forward', 1)

    def tower_listener_callback(self, msg):   
        
        opencv_image = cv2.resize(self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8"), (640, 360))
        cv2.imshow('tower', opencv_image)
        if(self.towerFrames>30):
            for value in ARUCO_DICT:
                arucoDict = cv2.aruco.Dictionary_get(value)
                arucoParams = cv2.aruco.DetectorParameters_create()
                (corners, ids, rejected) = cv2.aruco.detectMarkers(opencv_image, arucoDict,
                parameters=arucoParams)
                
                if ids:
                    print("tower", ids)
                

            self.towerFrames = 0
        self.towerFrames+=1
        cv2.waitKey(1)

    def chassis_listener_callback(self, msg):
        opencv_image = cv2.resize(self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8"), (640, 360))
        cv2.imshow('chassis', opencv_image)
        if(self.chassisFrames>30):
            for value in ARUCO_DICT:
                arucoDict = cv2.aruco.Dictionary_get(value)
                arucoParams = cv2.aruco.DetectorParameters_create()
                (corners, ids, rejected) = cv2.aruco.detectMarkers(opencv_image, arucoDict,
                parameters=arucoParams)
                
                if ids:
                    print("chassis", ids)
                

            self.chassisFrames = 0
        self.chassisFrames+=1
        cv2.waitKey(1)

    def infrared_listener_callback(self, msg):
        frame_bytes = msg.data  # Get the H.265 encoded bytes
        print("received")
        img = self.decode_h265_to_cv2_image(frame_bytes)
        
        if img is not None:
            cv2.imshow("Decoded Image", img)
            cv2.waitKey(1)

    def decode_h265_to_cv2_image(self,h265_bytes):
        """Decodes H.265 byte stream to a cv2 image (numpy array).

        Args:
            h265_bytes: Raw H.265 byte stream.
            width: Frame width.
            height: Frame height.

        Returns:
            A cv2 image (numpy array) or None if decoding fails.
        """
        ffmpeg_command = [
            'ffmpeg',
            '-f', 'hevc',
            '-framerate', '30',  # Adjust as needed
            '-x', '640',
            'y','480',
            '-i', '-',  # Read from stdin
            '-pix_fmt', 'bgr24',
            '-c:v', 'rawvideo',
            '-an',  # Disable audio
            '-y',  # Overwrite output file if it exists
            '-f', 'rawvideo',
            '-'
        ]

        process = sp.Popen(ffmpeg_command, stdin=sp.PIPE, stdout=sp.PIPE, stderr=sp.PIPE)
        
        raw_video, err = process.communicate(input=b''.join(h265_bytes))

        if err:
            print(f"FFmpeg error: {err.decode()}")
            return None

        if not raw_video:
            print("No output from FFmpeg, decoding failed.")
            return None

        image = np.frombuffer(raw_video, dtype=np.uint8).reshape((640, 480, 3))
        return image

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
    #print("PLease")
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
