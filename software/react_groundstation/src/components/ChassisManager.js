import React from 'react';

import ChassisControl from './ChassisControl.js';


function ChassisManager(props){
    //<Sliders id = "right"/>
    
    return (
        <div>
            
            <ChassisControl id = "left" ros = {props.ros}/>
            <ChassisControl id = "right" ros = {props.ros}/>

                
        </div>
        
    );
}

export default ChassisManager;