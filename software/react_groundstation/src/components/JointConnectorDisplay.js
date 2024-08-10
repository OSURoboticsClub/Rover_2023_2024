import React, { useRef , useState, useEffect} from 'react'
import ROSLIB from 'roslib'
let OFFSET_FROM_ORIGIN = 100
let SQUARE_CENTER = 150
let START_CIRCLE_DIAMETER = 30
let max_angle = 2
let CENTER_NODE = 0
let matrixMultiplication = require('matrix-multiplication')
let mul = matrixMultiplication()(3)
let JOINT_OFFSETS = [
    
    {"x":0,"y":-0.8},
    {"x":-0.4,"y":0},
    {"x":0,"y":-0.6},
    {"x":0.4,"y":0},
    {"x":0,"y":-0.5},
]

function populateArmJoints(setArmJoints,setCompleteMarker){
    var jointArr = []
    //ARBITRARY VALUES, REPLACE THIS WITH INFORMATION FROM THE ARM ROS MODULE 
    jointArr.push([0,0,0])
    var count = 0
    for (let coords of JOINT_OFFSETS){
        var prevEntry = jointArr[jointArr.length-1]
        
        if(count == CENTER_NODE)
            prevEntry = [0,0,0]
        console.log(prevEntry)

        var coord = [prevEntry[0]+coords["x"],prevEntry[1]+coords["y"],prevEntry[2]]
        jointArr.push(coord)
        count++
    }
    var offset = jointArr[CENTER_NODE]
    for (let coords of jointArr){
        coords[0] -= offset[0]
        coords[1] -= offset[1]
        
    }
    
    

    setArmJoints(jointArr)
    setCompleteMarker(true)
}

function calculateRotationMatrix(curPos,prevPos){
    var y_theta = -((curPos[0]-prevPos[0])/300)*max_angle //This is defined in the CSS rules for the wireframeWidget class
    var x_theta = -((curPos[1]-prevPos[1])/300)*max_angle //This is defined in the CSS rules for the wireframeWidget class
    
    var xRotationMatrix = [1,0,0,
                           0,Math.cos(x_theta),-Math.sin(x_theta),
                           0,Math.sin(x_theta),Math.cos(x_theta)]

    var yRotationMatrix = [Math.cos(y_theta),0,-Math.sin(y_theta),
                                    0,       1,        0,
                           Math.sin(y_theta),0,Math.cos(y_theta),]

    var finalRotationMatrix = mul(xRotationMatrix,yRotationMatrix)
    
    return finalRotationMatrix
    
}

function JointConnectorDisplay(props){
    const [mouseCurPos,setCurPos] = useState([])
    const [mousePrevPos,setPrevPos] = useState([])

    const [mouseDownNew,setDownNew] = useState(false)

    const [armJoints,setArmJoints] = useState([])
    const [buttonInterval,setButtonInterval] = useState(null)
    const [completeMarker, setCompleteMarker] = useState(false)

    var listener = new ROSLIB.Topic({
        ros : props.ros,
        name : '/rover_arm_controller/commands',
        messageType : 'std_msgs/msg/Float64MultiArray'
    })
    
     const printClick = () =>{
        setDownNew(true)
    }
    const printRelease = () =>{
        setDownNew(false)
    }
    const setArmJointsEmbed = (array) =>{
        setArmJoints(array)
    }
    
    useEffect(() => {
        
        populateArmJoints(setArmJoints,setCompleteMarker);
    },[])

    useEffect(()=>{
        listener.subscribe(function(message) {
            if(armJoints.length > 0){
                var positionCopy = armJoints
                //console.log(message.data);
                var joint = positionCopy[1]

                var jointLength = Math.sqrt(Math.pow(joint[1],2)+Math.pow(joint[2],2))
                joint[2] = jointLength*Math.cos((message.data[1]))
                joint[1] = jointLength*Math.sin((message.data[1]))
                console.log(joint)
                //joint[2] = 0.3
                
                setArmJoints(positionCopy)
            }
        });
    },[completeMarker])
    
    const trackMouse = (event) =>{
        setPrevPos(mouseCurPos)
        const bounds = event.currentTarget.getBoundingClientRect();
        const x = event.clientX - bounds.left;
        const y = event.clientY - bounds.top;
        setCurPos([x,y])
    // Now x and y are the relative coordinates.
        if(mouseDownNew){
            var finalRotationMatrix = calculateRotationMatrix(mouseCurPos,mousePrevPos)
            var translatedJointPoints = []
            
            for (var i = 0; i < armJoints.length; i++){
                
                var element = armJoints[i]
                
                var translatedPoint = mul(finalRotationMatrix,element)
                translatedJointPoints.push(translatedPoint)
                
            }
            
            setArmJoints(translatedJointPoints)

        }
    }
    var array = JSON.parse(JSON.stringify(armJoints))

    Math.degrees = function(radians) {
        return radians * 180 / Math.PI;
    }

    for(let i = 0; i < array.length; i++){
        if(i===0){
            array[0].push("darkred")
        }
        else if(i===array.length-1){
            array[i].push("darkblue")
        } else {
            array[i].push("green")
        }
        if(array[i+1]){
            var distance = Math.sqrt(
                Math.pow(array[i][0]*OFFSET_FROM_ORIGIN-array[i+1][0]*OFFSET_FROM_ORIGIN,2) + 
                Math.pow(array[i][1]*OFFSET_FROM_ORIGIN-array[i+1][1]*OFFSET_FROM_ORIGIN,2))
        
            array[i].push(<p class = "line" key = {30+i} style={{left: (-distance/2 + (array[i][0]*OFFSET_FROM_ORIGIN-array[i+1][0]*OFFSET_FROM_ORIGIN)/2)+SQUARE_CENTER+array[i+1][0]*OFFSET_FROM_ORIGIN,
                                                                   top: ((array[i][1]*OFFSET_FROM_ORIGIN-array[i+1][1]*OFFSET_FROM_ORIGIN)/2)+SQUARE_CENTER+array[i+1][1]*OFFSET_FROM_ORIGIN,
                                                                   width: distance,
                                                                   borderTopWidth: 5+array[i][2]*3,
                                                                   rotate: Math.degrees(Math.atan((array[i+1][1]*OFFSET_FROM_ORIGIN-array[i][1]*OFFSET_FROM_ORIGIN)/(array[i+1][0]*OFFSET_FROM_ORIGIN-array[i][0]*OFFSET_FROM_ORIGIN)))+"deg"}}/>)
                        }
    }
    
    array.sort(function(a, b) {
        return a[2] - b[2];
    })
    const renderedItems = [];
    for (let i = 0; i < array.length; i++){
        let joint = array[i]
        // REMINDER TO MAKE A LIST OF CONSTANTS, CHECK TO SEE IF JS AND CSS CAN SHARE CONSTANTS
        
        renderedItems.push(joint[4])
        renderedItems.push(<p class = "dot" key = {i} style={{left: SQUARE_CENTER-(START_CIRCLE_DIAMETER/2)+joint[0]*(OFFSET_FROM_ORIGIN),
                                                                top: SQUARE_CENTER-(START_CIRCLE_DIAMETER)/2+joint[1]*(OFFSET_FROM_ORIGIN),
                                                                borderRadius: START_CIRCLE_DIAMETER+joint[2]*(10),
                                                                height: START_CIRCLE_DIAMETER + joint[2]*10,
                                                                width: START_CIRCLE_DIAMETER + joint[2]*10,
                                                                borderWidth:(4+ joint[2]*3),
                                                                borderColor: joint[3]}}/>)//Filter does not work as intendeds
        
    }
    return (
        
        <div class = "wireframeWidget" onMouseMove={trackMouse} onMouseDown = {printClick} onMouseUp = {printRelease} onMouseLeave={printRelease}>
            {renderedItems}
        </div>
        
    )
}

export default JointConnectorDisplay