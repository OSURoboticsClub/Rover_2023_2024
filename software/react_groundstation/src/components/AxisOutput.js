import React from 'react';
import '../lib/controller.js';

function truncateDecimals(number) {
    return Math[number < 0 ? 'ceil' : 'floor'](number);
};


function SlidingColor(e,props){
    if((e.detail.directionOfMovement === "top" || e.detail.directionOfMovement === "bottom") && e.detail.stickMoved === props.id+"_stick"){
        var elem = document.getElementById(props.id)
        if(e.detail.directionOfMovement === "top"){
            elem.style.backgroundColor = "green";
        } else {
            console.log("bot")
            elem.style.backgroundColor = "red";
        }
    
        
        
        elem.innerHTML = truncateDecimals(e.detail.axisMovementValue*100 * -1) + "%";
        elem.style.width = Math.abs(truncateDecimals(e.detail.axisMovementValue*97 * -1)) + "%";
        console.log(e.detail)
    
    }
}

function AxisOutput(props){
    
    window.joypad.on('axis_move', function(e){SlidingColor(e,props)});
    return(
        <article className="driveOutput" id = {props.id}>0%</article>
    );
}

export default AxisOutput;