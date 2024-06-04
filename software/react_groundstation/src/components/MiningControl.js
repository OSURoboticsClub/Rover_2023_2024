import React,{useState,useEffect} from 'react';
import ROSLIB from 'roslib';
import { ARM_CONTROLLER_ID } from '../lib/constants';
//LINEAR_CONTROL_TOPIC = "mining/control/linear"
//COMPARTMENT_CHANGE_TOPIC = "mining/control/compartment"
//DRILL_CONTROL_TOPIC = "mining/drill/control"
function MiningControl(props){
    const LINEAR_CONTROL_TOPIC = new ROSLIB.Topic({
        ros: props.ros,
        name: "mining/control/linear",
        messageType: "rover2_control_interface/msg/DrillControlMessage"
    })
    const COMPARTMENT_CHANGE_TOPIC = new ROSLIB.Topic({
        ros: props.ros,
        name: "mining/control/compartment",
        messageType: "rover2_control_interface/msg/MiningControlMessage"
    })
    const DRILL_CONTROL_TOPIC = new ROSLIB.Topic({
        ros: props.ros,
        name: "mining/drill/control",
        messageType: "rover2_control_interface/msg/DrillControlMessage"
    })

    const compartmentChange = window.joypad.on('button_press', function(e){compartmentControl(e,props)})

    const compartmentControl = (e) => {
        
        
        if(!props.controlMining || ARM_CONTROLLER_ID !== e.detail.gamepad["id"]){
            return 
        }
        

        if(e.detail.buttonName === "button_0"){
            
        } else if(e.detail.buttonName === "button_1"){
            
        } else if(e.detail.buttonName === "button_2"){

        } else if(e.detail.buttonName === "button_3"){
            
        }

    
       
       
    }

    return(
        <h1> This is the Mining UI</h1>
    );
}
export default MiningControl