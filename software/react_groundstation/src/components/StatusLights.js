import React from 'react';
import Light from './Light.js';





function StatusLights(){
    //window.joypad.on('button_press', function(){flipColor("square1")});
    return (
        <div>
            <Light text = "Bogies" id = "bogies"/>
            <Light text = "Science" id = "science"/>
            
        </div>
        
    );
}

export default StatusLights;