import React from 'react';

import ChassisControl from './ChassisControl.js';


function ChassisManager(props){
    //<Sliders id = "right"/>
    //<ChassisControl id = "right" ros = {props.ros}/>
    return (
        <div>
            
            <ChassisControl id = {["left","right"]} ros = {props.ros}/>
            

                
        </div>
        
    );
}

export default ChassisManager;