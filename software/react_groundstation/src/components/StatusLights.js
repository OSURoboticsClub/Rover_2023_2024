
import Light from './LightControl.js';

function StatusLights(props){
    
    //window.joypad.on('button_press', function(){flipColor("square1")});
    return (
        <p>
            <Light text = "Chassis Cam" id = "chas_cam" controllerEvent = "button_press" button = "button_8" startColor = "green"/>
            <Light text = "Tower Cam" id = "tower_cam" controllerEvent = "button_press" button = "button_8"/>
            
        </p>
        
    );
}

export default StatusLights;