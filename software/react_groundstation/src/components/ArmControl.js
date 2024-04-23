import React,{useState} from 'react';
import JointConnectorDisplay from './JointConnectorDisplay';
import { ARM_CONTROLLER_ID } from '../lib/constants';


/* TODO: DISCOVER UNKNOWNS
*  What topics to publish JBJ(Joint-by-Joint) and IK (Inverse Kinematic) information to
*  What kind of numbers make sense to publish to said topics
*  How to receive joint information from the arm 
*/







function ArmControl(props){
    

    const [movementMode,setMovementMode] = useState("DISABLED");

    const armMovement = window.joypad.on('axis_move', function(e){armOutput(e,props)})
    
    
    

    const updateMovementMode = (update) =>{
        armMovement.unsubscribe()
        
        setMovementMode(update)
    }


    const armOutput = (e) => {
        //console.log(props.controlArm)
        if(!props.controlArm){
            return 
        }
        
        if((movementMode === "DISABLED" || ARM_CONTROLLER_ID !== e.detail.gamepad["id"]) || (e.detail.directionOfMovement !== "top" && e.detail.directionOfMovement !=="bottom")){
            return 
        }
        else if(movementMode === "JBJ"){
            console.log("JBJ")
        } else { //This is the IK control case
            console.log("IK")
        }
       
    }
    
    
    
    return(
        //has a camera
        //control via end-effector with IK and joint-to-joiny control (JTJ control)
        //wireframe 3d (use euler angles with 3d rotation matricies for the 3 normal values)
        
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