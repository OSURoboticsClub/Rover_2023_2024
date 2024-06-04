import ROSLIB from 'roslib'
import React,{useState,useEffect} from 'react';

function GripperControl(props){
    const GRIPPER_CONTROL_TOPIC = new ROSLIB.Topic({
        ros: props.ros,
        name: "gripper/control",
        messageType: "rover2_control_interface/msg/GripperControlMessage"
    })

    

}

export default GripperControl;