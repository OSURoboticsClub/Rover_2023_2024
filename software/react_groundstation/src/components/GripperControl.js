import ROSLIB from 'roslib'
import React,{useState,useEffect} from 'react';
import { ARM_CONTROLLER_ID } from '../lib/constants';

function gripperAxisOutput(e,topic){
    
    if(ARM_CONTROLLER_ID !== e.detail.gamepad["id"] || (e.detail.directionOfMovement)){
        return ;
    }
    var moveValue = e.detail.axisMovementValue
    if(e.detail.axis == 4){
        moveValue *= -1
    }
    publishMessage(Math.round(moveValue*300),false,false,false,topic)
}

function gripperButtonOutput(e,topic){
    
    if(ARM_CONTROLLER_ID !== e.detail.gamepad["id"]){
        
        return ;
    }
    
    if(e.detail.buttonName == "button_4"){
        publishMessage(0,false,false,true,topic)
    } else if(e.detail.buttonName == "button_5"){
        publishMessage(0,false,true,false,topic)
    } else if(e.detail.buttonName == "button_16"){
        publishMessage(0,true,false,false,topic)
    }
    
}

function publishMessage(moveValue,shouldHome,light,laser,topic){
    var data = new ROSLIB.Message({
        target: moveValue,
        should_home: shouldHome,
        toggle_light: light,
        toggle_laser: laser
    });
    console.log('msg', data)
    topic.publish(data);
}

function GripperControl(props){
    const GRIPPER_CONTROL_TOPIC = new ROSLIB.Topic({
        ros: props.ros,
        name: "gripper/control",
        messageType: "rover2_control_interface/msg/GripperControlMessage"
    })
    console.log("ugh")
    const gripperMovement = window.joypad.on('axis_move', function(e){gripperAxisOutput(e,GRIPPER_CONTROL_TOPIC);});
    const gripperUtils = window.joypad.on('button_press', function(e){gripperButtonOutput(e,GRIPPER_CONTROL_TOPIC);});

}

export default GripperControl;