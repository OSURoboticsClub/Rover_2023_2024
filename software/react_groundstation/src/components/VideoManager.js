import React from 'react';
import ROSLIB from 'roslib';
import Light from './Light.js';
import CameraPanTilt from './CameraPanTilt.js';

function VideoManager(props){
    let ros = props.ros

    var chassis_listener = new ROSLIB.Topic({
        ros : ros,
        name : '/cameras/chassis/image_640x360/compressed',
        messageType : 'sensor_msgs/CompressedImage'
      });
    
      chassis_listener.subscribe(function(message) {
        document.getElementById("chassis_image").src = "data:image/png;base64," + message.data
        
      });
    
      var tower_listener = new ROSLIB.Topic({
        ros : ros,
        name : '/cameras/main_navigation/image_640x360/compressed',
        messageType : 'sensor_msgs/CompressedImage'
        
        
      });
    
      tower_listener.subscribe(function(message) {
        document.getElementById("tower_image").src = "data:image/png;base64," + message.data
      });

    return(
        <div>
            <CameraPanTilt ros = {props.ros} button_switch = "button_6" button_center = "button_7" buttons_move = {["button_3",//"button_12"
                                                                                                                   "button_0",//"button_13"
                                                                                                                   "button_1",//"button_14"
                                                                                                                   "button_2"]}/*"button_15"*//>
            <Light text = "Chassis Cam" id = "chas_cam" controllerEvent = "button_press" button = "button_6" startColor = "green"/>
            <Light text = "Tower Cam" id = "tower_cam" controllerEvent = "button_press" button = "button_6"/>
            <h1>Tower cam</h1>
            <div><img src="" id="tower_image"/></div>
            <h1>Chassis cam</h1>
            <div><img src="" id="chassis_image"/></div>
        </div>
    );
}

export default VideoManager;