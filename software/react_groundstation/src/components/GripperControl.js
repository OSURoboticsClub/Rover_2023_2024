import ROSLIB from 'roslib'
import React,{useState,useEffect} from 'react';
import { ARM_CONTROLLER_ID } from '../lib/constants';

// function gripperAxisOutput(e,topic){
    
//     if(ARM_CONTROLLER_ID !== e.detail.gamepad["id"] || (e.detail.directionOfMovement)){
//         return ;
//     }
//     var moveValue = e.detail.axisMovementValue
//     if(e.detail.axis == 4){
//         moveValue *= -1
//     }
//     publishMessage(Math.round(moveValue*300),false,false,false,topic)
// }



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
    const [buttonInterval,setButtonInterval] = useState(null)
    const [buttonStatus,setButtonStatus] = useState([false,false])
    const GRIPPER_CONTROL_TOPIC = new ROSLIB.Topic({
        ros: props.ros,
        name: "gripper/control",
        messageType: "rover2_control_interface/msg/GripperControlMessage"
    })

    const gripperButtonOutput = (e) => {
    
        if(ARM_CONTROLLER_ID !== e.detail.gamepad["id"]){
            
            return ;
        }

        if(e.detail.buttonName == "button_1"){
            setButtonStatus([buttonStatus[0],true])
        }
        if(e.detail.buttonName == "button_3"){
            setButtonStatus([true,buttonStatus[1]])
        }
        if(e.detail.buttonName == "button_4"){
            publishMessage(0,false,false,true,GRIPPER_CONTROL_TOPIC)
        } else if(e.detail.buttonName == "button_5"){
            publishMessage(0,false,true,false,GRIPPER_CONTROL_TOPIC)
        } else if(e.detail.buttonName == "button_16"){
            publishMessage(0,true,false,false,GRIPPER_CONTROL_TOPIC)
        }
        
    }

    const gripperButtonReleased = (e) =>{
        if(ARM_CONTROLLER_ID !== e.detail.gamepad["id"]){
            
            return ;
        }

        if(e.detail.buttonName == "button_1"){
            setButtonStatus([buttonStatus[0],false])
        }
        if(e.detail.buttonName == "button_3"){
            setButtonStatus([false,buttonStatus[1]])
        }
    }
    useEffect(() => {
        //const gripperMovement = window.joypad.on('axis_move', function(e){gripperAxisOutput(e,GRIPPER_CONTROL_TOPIC);});
        const gripperUtils = window.joypad.on('button_press', function(e){gripperButtonOutput(e);});
        const dpadReleased = window.joypad.on('button_release', function(e){gripperButtonReleased(e)})
    },[])
    useEffect(() => { //Must include these useEffects to unsub from chassis control listener to prevent CPU and memory leaks and overruns
        clearInterval(buttonInterval)
        //console.log(dpadStatus)
        
        
        if(buttonStatus.includes(true)){
            setButtonInterval(
                setInterval(() => {
                    
                    if(buttonStatus[0]){
                        publishMessage(300,false,false,false,GRIPPER_CONTROL_TOPIC)
                    }
                    if(buttonStatus[1]){
                        publishMessage(-300,false,false,false,GRIPPER_CONTROL_TOPIC)
                    }
                    
                }, 15)
            )
        }
    },[buttonStatus])
}

export default GripperControl;