import React,{useState,useEffect} from 'react';
import ROSLIB from 'roslib';
import { ARM_CONTROLLER_ID } from '../lib/constants';
//LINEAR_CONTROL_TOPIC = "mining/control/linear"
//COMPARTMENT_CHANGE_TOPIC = "mining/control/compartment"
//DRILL_CONTROL_TOPIC = "mining/drill/control"
const MAX_DRILL_RPS = 100


function MiningControl(props){
    const [compartment,setCompartment] = useState(0)

    const COMPARTMENT_CHANGE_TOPIC = new ROSLIB.Topic({
        ros: props.ros,
        name: "/drill/compartment",
        messageType: "std_msgs/msg/Int16"
    })
    
    const MAIN_ACTUATOR_CONTROL_TOPIC = new ROSLIB.Topic({
        ros: props.ros,
        name: "/scimech_control/main_actuator",
        messageType: "rover2_control_interface/msg/DriveControlMessage"
    })

    const FLEXINOL_CONTROL_TOPIC = new ROSLIB.Topic({
        ros: props.ros,
        name: "/scimech_control/flexinol",
        messageType: "rover2_control_interface/msg/DriveControlMessage"
    })
    
    const SECONDARY_ACTUATOR_CONTROL_TOPIC = new ROSLIB.Topic({
        ros: props.ros,
        name: "/scimech_control/secondary_actuator",
        messageType: "rover2_control_interface/msg/DriveControlMessage"
    })

    const DRILL_CONTROL_TOPIC = new ROSLIB.Topic({
        ros: props.ros,
        name: "/drill/control",
        messageType: "std_msgs/msg/Float32"
    })
    
    const publishCompartmentChange = (position) => {
        const data = new ROSLIB.Message({
            data: position
        })
        setCompartment(position)
        console.log(data)
        COMPARTMENT_CHANGE_TOPIC.publish(data)
    }

    const publishDrill = (moveValue) => {
        const data = new ROSLIB.Message({
            data: moveValue*MAX_DRILL_RPS
        })
        console.log(data)
        
        DRILL_CONTROL_TOPIC.publish(data)
    }

    const publishFlexinol= (moveValue) => {
        
        const data = new ROSLIB.Message({
            first_motor_speed: Math.round(65535*moveValue*0.05),
            first_motor_direction: false
        })
        console.log(data)
        FLEXINOL_CONTROL_TOPIC.publish(data)
    }
    
    
    const publishSecondaryActuatorControl= (moveValue) => {
        var direction = false
        if (moveValue < 0){
            direction = true
            moveValue = -1 * moveValue
        }
        const data = new ROSLIB.Message({
            first_motor_speed: Math.round(65535*moveValue),
            first_motor_direction: direction
        })
        console.log(data)
        SECONDARY_ACTUATOR_CONTROL_TOPIC.publish(data)
    }
    
    const publishMainActuatorControl= (moveValue) => {
        var direction = false
        if (moveValue < 0){
            direction = true
            moveValue = -1 * moveValue
        }
        const data = new ROSLIB.Message({
            first_motor_speed: Math.round(65535*moveValue),
            first_motor_direction: direction
        })
        console.log(data)
        MAIN_ACTUATOR_CONTROL_TOPIC.publish(data)
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
        
        if(e.detail.directionOfMovement && e.detail.stickMoved === "left_stick"){
            publishMainActuatorControl(e.detail.axisMovementValue)
        }
        if(e.detail.directionOfMovement && e.detail.stickMoved === "right_stick"){
            publishSecondaryActuatorControl(e.detail.axisMovementValue)
        }
        if (!e.detail.directionOfMovement && e.detail.axis === 5){
            publishDrill(e.detail.axisMovementValue)
        }
        if (!e.detail.directionOfMovement && e.detail.axis === 4){
            publishFlexinol(e.detail.axisMovementValue)
            console.log("LMAMAMAMMAMA")
        }

    }
    const changeServoBlock = () => {
        publishCompartmentChange(550,"block")
    };
    const changeServoCollect = () => {
        publishCompartmentChange(1500,"open")
    };
    const changeServoClean = () => {
        publishCompartmentChange(2500,"clean")
    };
    useEffect(()=>{
        // const compartmentChange = window.joypad.on('button_press', function(e){compartmentControl(e)})
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
            <h3>
                <button onClick={changeServoBlock}>
                    Change servo block
                </button>
            </h3>
            <h3>
                <button onClick={changeServoCollect}>
                    Change servo collect
                </button>
            </h3>
            <h3>
                <button onClick={changeServoClean}>
                    Change servo cleanup
                </button>
            </h3>
            
                

        </div>
    );
}
export default MiningControl