import React,{useState} from 'react';

import ChassisControl from './ChassisControl.js';
import Slider from './Slider.js';
import CameraPanTilt from './CameraPanTilt.js';


function ChassisManager(props){
    //<Sliders id = "right"/>
    //<ChassisControl id = "right" ros = {props.ros}/>
    
    const [throttle,setThrottle] = useState(1)
    
    return (
        <div>
            
            <ChassisControl id = {["left","right"]} ros = {props.ros} throttle = {throttle}/>
            <Slider setThrottle = {setThrottle}/>

            <CameraPanTilt  ros = {props.ros} button_switch = "button_8" button_center = "button_9" buttons_move = {["button_2",//"button_12"
                                                                                                                                                  "button_0",//"button_13"
                                                                                                                                                  "button_1",//"button_14*"
                                                                                                                                                  "button_3"]}/*"button_15"*//>
            
        </div>
        
    );
}

export default ChassisManager;