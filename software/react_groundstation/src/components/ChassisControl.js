import React,{useState} from 'react';
import ROSLIB from 'roslib';


var leftYAxis = 0
var rightYAxis = 0

function truncateDecimals(number) {
    return Math[number < 0 ? 'ceil' : 'floor'](number);
};

function driveOutput(e,props,topic,setLeftOutput,setRightOutput, setLeftColor, setRightColor,setLeftWidth,setRightWidth){
   
    var motorVals = {}
    for(var i = 0; i < 2; i++){
        if((e.detail.directionOfMovement === "top" || e.detail.directionOfMovement === "bottom") && e.detail.stickMoved === props.id[i]+"_stick"){
            
            //var elem = document.getElementById(props.id[i])

            if(e.detail.directionOfMovement === "top" && props.id[i] === "left"){
                setLeftColor("green")
            } else if(e.detail.directionOfMovement === "top" && props.id[i] === "right"){
                setRightColor("green")
            } else if(e.detail.directionOfMovement === "bottom" && props.id[i] === "left"){
                setLeftColor("red")
            } else if(e.detail.directionOfMovement === "bottom" && props.id[i] === "right"){
                setLeftColor("red")
            }
            if(props.id[i] === "left"){
                
                leftYAxis = -1 * e.detail.axisMovementValue
                setLeftOutput(truncateDecimals(leftYAxis*100)+ "%")
                setLeftWidth(truncateDecimals(Math.abs(leftYAxis*97))+ "%")
            } else {
                rightYAxis = -1 * e.detail.axisMovementValue
                setRightOutput(truncateDecimals(rightYAxis*100)+ "%")
                setRightWidth(truncateDecimals(Math.abs(rightYAxis*97))+ "%")
            }

            
            //elem.innerHTML = truncateDecimals(e.detail.axisMovementValue*100 * -1) + "%";
            //elem.style.width = Math.abs(truncateDecimals(e.detail.axisMovementValue*97 * -1)) + "%";
            
        }
        

    }

        
        
        //console.log("Left axis: %f Right axis: %f",leftYAxis,rightYAxis)
        motorVals.left = leftYAxis
        motorVals.right = rightYAxis
     
        sendDriveMessage(props,topic,leftYAxis,rightYAxis)
    
}

function sendDriveMessage(props,topic,leftYAixs,rightYAxis){
 
    const data = new ROSLIB.Message({
        controller_present: true,
        ignore_drive_control: false,
        drive_twist: {
                linear : {
                  x : (leftYAixs + rightYAxis) / 2.0,
                  y : 0,
                  z : 0
                },
                angular : {
                  x : 0,
                  y : 0,
                  z : (rightYAxis - leftYAixs) / 2.0
                }
            }
    })
    console.log(data)
    topic.publish(data)
}

function ChassisControl(props){
    const [leftOutput,setLeftOutput] = useState(0)
    const [rightOutput,setRightOutput] = useState(0)

    const [leftColor,setLeftColor] = useState("red")
    const [rightColor,setRightColor] = useState("red")
    
    const [leftWidth,setLeftWidth] = useState(0)
    const [rightWidth,setRightWidth] = useState(0)
    
    const topic = new ROSLIB.Topic({
        ros: props.ros,
        name: "command_control/ground_station_drive",
        messageType: "rover2_control_interface/msg/DriveCommandMessage"
    })
    
 
    window.joypad.on('axis_move', function(e){driveOutput(e,props,topic,setLeftOutput,setRightOutput,
                                                                        setLeftColor, setRightColor,
                                                                        setLeftWidth, setRightWidth)});
    
    
    return(
        <div>
            <article className="driveOutput" id = {props.id[0]} style={{backgroundColor: leftColor, width: leftWidth}}>{leftOutput}</article>
            <article className="driveOutput" id = {props.id[1]} style={{backgroundColor: rightColor, width:rightWidth}}>{rightOutput}</article>
        </div>
        
    );
}

export default ChassisControl;