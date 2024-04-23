
import Light from './LightControl.js';

function StatusLights(props){
    
    //window.joypad.on('button_press', function(){flipColor("square1")});
    return (
        <p>
            <Light text = "Bogies" id = "bogies"/>
            <Light text = "Science" id = "science"/>
            
        </p>
        
    );
}

export default StatusLights;