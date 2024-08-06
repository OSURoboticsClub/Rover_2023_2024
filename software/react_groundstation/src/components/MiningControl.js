import React,{useState,useEffect} from 'react';
import ROSLIB from 'roslib';
import { ARM_CONTROLLER_ID } from '../lib/constants';
//LINEAR_CONTROL_TOPIC = "mining/control/linear"
//COMPARTMENT_CHANGE_TOPIC = "mining/control/compartment"
//DRILL_CONTROL_TOPIC = "mining/drill/control"



function MiningControl(props){
    const [compartment,setCompartment] = useState(0)

    const COMPARTMENT_CHANGE_TOPIC = new ROSLIB.Topic({
        ros: props.ros,
        name: "/mining/control/compartment",
        messageType: "rover2_control_interface/msg/MiningControlMessage"
    })
    
    const LINEAR_CONTROL_TOPIC = new ROSLIB.Topic({
        ros: props.ros,
        name: "/mining/control/linear",
        messageType: "rover2_control_interface/msg/DrillControlMessage"
    })
    
    const DRILL_CONTROL_TOPIC = new ROSLIB.Topic({
        ros: props.ros,
        name: "/mining/drill/control",
        messageType: "rover2_control_interface/msg/DrillControlMessage"
    })
    
    const publishCompartmentChange = (position) => {
        const data = new ROSLIB.Message({
            compartment: position
        })
        setCompartment(position)
        console.log(data)
        COMPARTMENT_CHANGE_TOPIC.publish(data)
    }
    
    const publishLinearControl = (moveValue) => {
        var dir = false
        if(moveValue < 0)
            dir = true
        const data = new ROSLIB.Message({
            speed: Math.round(65535*Math.abs(moveValue)), 
            direction: dir
            
        })
        console.log(data)
        LINEAR_CONTROL_TOPIC.publish(data)
    }
    
    const publishDrillControl= (moveValue) => {
        const data = new ROSLIB.Message({
            speed: Math.round(65535*moveValue),
            direction: false
        })
        console.log(data)
        DRILL_CONTROL_TOPIC.publish(data)
    }

    const compartmentControl = (e) => {
        
        if(!props.controlMining || ARM_CONTROLLER_ID !== e.detail.gamepad["id"]){
            return 
        }
        if(e.detail.buttonName === "button_0"){
            publishCompartmentChange(0)
        } else if(e.detail.buttonName === "button_1"){
            publishCompartmentChange(1)
        } else if(e.detail.buttonName === "button_2"){
            publishCompartmentChange(2)
        } else if(e.detail.buttonName === "button_3"){
            publishCompartmentChange(3)
        }
    }

    const joystickHandling = (e) => {
        if(!props.controlMining || ARM_CONTROLLER_ID !== e.detail.gamepad["id"]){
            
            return 
        }
        if(e.detail.directionOfMovement && e.detail.stickMoved == "left_stick"){
            publishLinearControl(e.detail.axisMovementValue)
        }
        if(!e.detail.directionOfMovement){
            publishDrillControl(e.detail.axisMovementValue)
        }

    }

    useEffect(()=>{
        const compartmentChange = window.joypad.on('button_press', function(e){compartmentControl(e)})
        const controlDrillAndLinear = window.joypad.on('axis_move',function(e){joystickHandling(e)})
    },[])
    
    return(
        <div>
            <h1> 
                This is the Mining UI
            </h1>
            <h2>
                Current Compartment: {compartment}
            </h2>

        </div>
    );
}
export default MiningControl