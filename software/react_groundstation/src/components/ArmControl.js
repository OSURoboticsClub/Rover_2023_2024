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
let seqID = 0

function translateJoystickMsg(e){
    //MAKE SURE CONTROLLER INPUT IS VALIDATED BEFORE CALLING
    var stickArr = [0,0,0,0,0,0,0,0]
    
    
    if(e.detail.axis === 0 && (e.detail.directionOfMovement === "left" || e.detail.directionOfMovement === "right"))
        stickArr[0] = e.detail.axisMovementValue * -1
        
    else if(e.detail.axis === 1 && (e.detail.directionOfMovement === "bottom" || e.detail.directionOfMovement === "top"))
        stickArr[1] = e.detail.axisMovementValue

    else if(e.detail.directionOfMovement === null && e.detail.axis === 4)
        stickArr[2] = e.detail.axisMovementValue
    
    else if(e.detail.axis === 2 && (e.detail.directionOfMovement === "left" || e.detail.directionOfMovement === "right"))
        stickArr[3] = e.detail.axisMovementValue

    else if(e.detail.axis === 3 && (e.detail.directionOfMovement === "bottom" || e.detail.directionOfMovement === "top"))
        stickArr[4] = e.detail.axisMovementValue * -1

    else if(e.detail.directionOfMovement === null && e.detail.axis === 5)
        stickArr[5] = e.detail.axisMovementValue

    //GET DPAD VALUES (THEY ARE BUTTONS FFS    (ﾉ °益°)ﾉ 彡 ┻━┻)
    
    return stickArr
}
function translateButtonPresses(dpadStatus){
    var stickArr = [0,0,0,0,0,0,0,0]

    if(dpadStatus[0])
        stickArr[7] = -1

    else if(dpadStatus[1])
        stickArr[7] = 1

    else if(dpadStatus[2])
        stickArr[6] = -1

    else if(dpadStatus[3])
        stickArr[6] = 1

    return stickArr
}


function publishArmControl(topic,inputArr){
    seqID+=1
    var buttonArr = [0,0,0,0,0,0,0,0,0,0,0]
    const data = new ROSLIB.Message({
        header: {
            
            stamp : {
              sec : 0,
              nsec : 0
            },
            frame_id: ""
        },
        
        axes: inputArr,
        buttons: buttonArr
        
    })
    console.log(inputArr)
    topic.publish(data)
}



function ArmControl(props){
    
    

    const [movementMode,setMovementMode] = useState("DISABLED");
    const [dpadStatus,setDpadStatus] = useState([false,false,false,false])
    const [buttonInterval,setButtonInterval] = useState(null)

    const armJoyMovement = window.joypad.on('axis_move', function(e){armOutput(e)})
    const dpadPressed = window.joypad.on('button_press', function(e){
        console.log(e.detail.buttonName)
        if(e.detail.gamepad["id"] !== ARM_CONTROLLER_ID)
            return
        if((movementMode === "DISABLED" || e.detail.gamepad["id"] !== ARM_CONTROLLER_ID) ){
            return 
        }

        if(e.detail.buttonName === "button_12")
            setDpadStatus([true,false,false,false])
        else if(e.detail.buttonName === "button_13")
            setDpadStatus([false,true,false,false])
        else if(e.detail.buttonName === "button_14")
            setDpadStatus([false,false,true,false])
        else if(e.detail.buttonName === "button_15")
            setDpadStatus([false,false,false,true])
        
    })
    
    const dpadReleased = window.joypad.on('button_release', function(e){
        
        if(e.detail.gamepad["id"] !== ARM_CONTROLLER_ID)
            return
        if(e.detail.buttonName === "button_12" || e.detail.buttonName === "button_13" || e.detail.buttonName === "button_14" || e.detail.buttonName === "button_15")
        setDpadStatus([false,false,false,false])
    })
    

    const topic = new ROSLIB.Topic({
        ros: props.ros,
        name: "/joy",
        messageType: "sensor_msgs/Joy"
    })
    
    //Below is the most disgusting code I have ever written... I am become death
    const updateMovementMode = (update) =>{
        armJoyMovement.unsubscribe()
        dpadPressed.unsubscribe()
        dpadReleased.unsubscribe()
        setMovementMode(update)
    }
    
    useEffect(() => { //Must include these useEffects to unsub from chassis control listener to prevent CPU and memory leaks and overruns
        clearInterval(buttonInterval)
        console.log(dpadStatus)
        dpadPressed.unsubscribe()
        dpadReleased.unsubscribe()
        armJoyMovement.unsubscribe()
        
        if(dpadStatus.includes(true)){
            setButtonInterval(
                setInterval(() => {
                    var inputArr = translateButtonPresses(dpadStatus)
                    publishArmControl(topic,inputArr)
                    
                }, 15)
            )
        }
    },[dpadStatus])

    useEffect(() => { //Must include these useEffects to unsub from chassis control listener to prevent CPU and memory leaks and overruns
        dpadPressed.unsubscribe()
        dpadReleased.unsubscribe()
        armJoyMovement.unsubscribe()
    },[buttonInterval])

    
    const armOutput = (e) => {
        console.log(props.controlArm)
        if(!props.controlArm){
            return 
        }
        
        if((movementMode === "DISABLED" || e.detail.gamepad["id"] !== ARM_CONTROLLER_ID) ){
            return 
        }
 
        
        var inputArr = translateJoystickMsg(e)
        
        publishArmControl(topic,inputArr)
        
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