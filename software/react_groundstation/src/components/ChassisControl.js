import React,{useState,useEffect} from 'react';
import ROSLIB from 'roslib';
import {DRIVE_CONTROLLER_ID} from '../lib/constants.js';


var leftYAxis = 0
var rightYAxis = 0
var colors = {left: 0, right:0}
var output = {left: "0%", right:"%"}
var width = {left:"0%", right: "0%"}

function truncateDecimals(number) {
    return Math[number < 0 ? 'ceil' : 'floor'](number);
};

function driveOutput(e,props,updateChassisState){
    
    if(DRIVE_CONTROLLER_ID !== e.detail.gamepad["id"] || (e.detail.directionOfMovement !== "top" && e.detail.directionOfMovement !== "bottom")){
        return ;
    }
    
    for(var i = 0; i < 2; i++){
        //This if statement is required as the triggers are classified as sticks, and therefore will have their own stick_moved property
        if(e.detail.stickMoved === props.id[i]+"_stick"){
            
            //var elem = document.getElementById(props.id[i])

            if(e.detail.directionOfMovement === "top" && props.id[i] === "left"){
                colors.left = "green"
                
            } else if(e.detail.directionOfMovement === "top" && props.id[i] === "right"){
                colors.right = "green"
                
            } else if(e.detail.directionOfMovement === "bottom" && props.id[i] === "left"){
                colors.left = "red"
                
            } else if(e.detail.directionOfMovement === "bottom" && props.id[i] === "right"){
                colors.right = "red"
                
            }
            if(props.id[i] === "left"){
                
                leftYAxis = e.detail.axisMovementValue
                output.left = truncateDecimals(leftYAxis*props.throttle*-100)+ "%"
                width.left = truncateDecimals(Math.abs(leftYAxis*props.throttle*-97))+ "%"
                
            } else {
                rightYAxis = e.detail.axisMovementValue
                output.right = truncateDecimals(rightYAxis*props.throttle*-100)+ "%"
                width.right = truncateDecimals(Math.abs(rightYAxis*props.throttle*-97))+ "%"
                
            }

            
          
            
        }
        
    }

    updateChassisState({
        leftOutput: output.left,
        rightOutput: output.right,
        leftColor: colors.left,
        rightColor: colors.right,
        leftWidth: width.left,
        rightWidth: width.right
    })
    
}

function sendDriveMessage(topic,throttle){
    var adjustedYLeft = -1*throttle*leftYAxis
    var adjustedYRight = -1*throttle*rightYAxis
    const data = new ROSLIB.Message({
        controller_present: true,
        ignore_drive_control: false,
        drive_twist: {
                linear : {
                  x : (adjustedYLeft + adjustedYRight) / 2.0,
                  y : 0,
                  z : 0
                },
                angular : {
                  x : 0,
                  y : 0,
                  z : (adjustedYRight - adjustedYLeft) / 2.0
                }
            }
    })
    
    
    topic.publish(data)

} 

function ChassisControl(props){
    let [chassisState,updateChassisState] = useState({
        leftOutput: 0,
        rightOutput: 0,
        leftColor: "red",
        rightColor: "red",
        leftWidth: 0,
        rightWidth: 0
    })
    let [lightState,updateLightState] = useState(0)
    
    const controlTopic = new ROSLIB.Topic({
        ros: props.ros,
        name: "command_control/ground_station_drive",
        messageType: "rover2_control_interface/msg/DriveCommandMessage"
    })
    const lightTopic = new ROSLIB.Topic({
        ros: props.ros,
        name: "/tower/light/control",
        messageType: "rover2_control_interface/msg/LightControlMessage"
    })

    const chassisControls = window.joypad.on('axis_move', function(e){driveOutput(e,props,updateChassisState);});
    const lightToggle = window.joypad.on('button_press', function(e){
        console.log(e.detail.gamepad["id"])
        if(DRIVE_CONTROLLER_ID === e.detail.gamepad["id"] && e.detail.buttonName === "button_16"){
        
            if(lightState === 0){
                updateLightState(2)
            }
            else {
                updateLightState(0)
            }
        }
    });
    useEffect(() => {
        const data = new ROSLIB.Message({
            light_mode: lightState
        })
        console.log(data)
        lightTopic.publish(data)
    },[lightState])
    useEffect(() => { //Must include these useEffects to unsub from chassis control listener to prevent CPU and memory leaks and overruns
        //chassisControls.unsubscribe() 
        sendDriveMessage(controlTopic,props.throttle)
        
    },[chassisState])
    
    
   
    
    return(
        <div>
            <article className="driveOutput" id = {props.id[0]} style={{backgroundColor: chassisState.leftColor, width: chassisState.leftWidth}}>{chassisState.leftOutput}</article>
            <article className="driveOutput" id = {props.id[1]} style={{backgroundColor: chassisState.rightColor, width: chassisState.rightWidth}}>{chassisState.rightOutput}</article>
        </div>
        
    );
}

export default ChassisControl;