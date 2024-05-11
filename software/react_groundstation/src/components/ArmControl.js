import React,{useState,useEffect} from 'react';
import JointConnectorDisplay from './JointConnectorDisplay';
import { ARM_CONTROLLER_ID } from '../lib/constants';
import ROSLIB from 'roslib'


/* TODO: DISCOVER UNKNOWNS
*  GitHub Repo: https://github.com/PreetDhami/rover_arm_capstone_2024/tree/inverse-kinematics
----------------------------------------------------------------------------------------------------
*  
* What topic to publish information to nodes spinning on rover to control arm? "/joy"
* Commands already grabbed from rover?
*/
/*
enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 3,
  RIGHT_STICK_Y = 4,
  RIGHT_TRIGGER = 5,
  D_PAD_X = 6,
  D_PAD_Y = 7
};
enum Button
{
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,
  CHANGE_VIEW = 6,
  MENU = 7,
  HOME = 8,
  LEFT_STICK_CLICK = 9,
  RIGHT_STICK_CLICK = 10
};
*/
function translateJoystickMsg(e){
    //MAKE SURE CONTROLLER INPUT IS VALIDATED BEFORE CALLING
    var stickArr = [0,0,0,0,0,0,0,0]
    

    if(e.detail.stickMoved === "left_stick" && (e.detail.direction === "left" || e.detail.direction === "right"))
        stickArr[0] = e.detail.axisMovementValue

    if(e.detail.stickMoved === "left_stick" && (e.detail.direction === "bottom" || e.detail.direction === "top"))
        stickArr[1] = e.detail.axisMovementValue

    if(e.detail.directionOfMovement === null && e.detail.axis === 4)
        stickArr[2] = e.detail.axisMovementValue
    
    if(e.detail.stickMoved === "right_stick" && (e.detail.direction === "left" || e.detail.direction === "right"))
        stickArr[3] = e.detail.axisMovementValue

    if(e.detail.stickMoved === "right_stick" && (e.detail.direction === "bottom" || e.detail.direction === "top"))
        stickArr[4] = e.detail.axisMovementValue

    if(e.detail.directionOfMovement === null && e.detail.axis === 5)
        stickArr[5] = e.detail.axisMovementValue

    //GET DPAD VALUES (THEY ARE BUTTONS FFS    (ﾉ °益°)ﾉ 彡 ┻━┻)
}




function ArmControl(props){
    

    const [movementMode,setMovementMode] = useState("DISABLED");

    const armMovement = window.joypad.on('axis_move', function(e){armOutput(e,props)})
    let seqID = 0

    const topic = new ROSLIB.Topic({
        ros: props.ros,
        name: "/joy",
        messageType: "sensor_msgs/msg/joy"
    })
    

    const updateMovementMode = (update) =>{
        armMovement.unsubscribe()
        
        setMovementMode(update)
    }


    const armOutput = (e) => {
        //console.log(props.controlArm)
        if(!props.controlArm){
            return 
        }
        
        if((movementMode === "DISABLED" || ARM_CONTROLLER_ID !== e.detail.gamepad["id"]) ){
            return 
        }
 
        if(movementMode === "JBJ"){
            
        } else { //This is the IK control case
            
        }
        var joyArr = translateJoystickMsg(e)
        var buttonArr = [0,0,0,0,0,0,0,0,0,0,0]
        const data = new ROSLIB.Message({
            header: {
                seq : seqID,
                stamp : {
                  sec : 0,
                  nsec : 0
                },
                frame_id: ""
            },
            
            axes: joyArr,
            buttons: buttonArr
            
        })
        seqID+=1
        topic.publish(data)
    }
    
    
    
    return(
        //has a camera
        //control via end-effector with IK and joint-to-joiny control (JTJ control)
        //wireframe 3d (use euler angles with 3d rotation matricies for the 3 normal vectors)
        
        <div class = "arm">
            
            <h1> Control Mode: 
                <select name="controlMode"
                    value={movementMode}
                    onChange={e => updateMovementMode(e.target.value)}
                    id="controlMode"
                    required>
                    <option value = "DISABLED">DISABLED</option>
                    <option value="JBJ">JBJ</option>
                    <option value="IK">IK</option>
                </select>
            </h1>
            
            <div class = "jointInformation"></div>
            
            <JointConnectorDisplay ros = {props.ros}/>
            
            <div class = "armCameraFeed"></div>

        </div>
        
    );
}
export default ArmControl;