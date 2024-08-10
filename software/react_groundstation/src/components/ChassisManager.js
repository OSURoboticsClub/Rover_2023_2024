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

            
            
        </div>
        
    );
}

export default ChassisManager;