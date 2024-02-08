import React from 'react';
import ROSLIB from 'roslib';


function truncateDecimals(number) {
    return Math[number < 0 ? 'ceil' : 'floor'](number);
};

function slidingColor(e,props){
    if((e.detail.directionOfMovement === "top" || e.detail.directionOfMovement === "bottom") && e.detail.stickMoved === props.id+"_stick"){
        var elem = document.getElementById(props.id)
        if(e.detail.directionOfMovement === "top"){
            elem.style.backgroundColor = "green";
        } else {
            
            elem.style.backgroundColor = "red";
        }
    

        elem.innerHTML = truncateDecimals(e.detail.axisMovementValue*100 * -1) + "%";
        elem.style.width = Math.abs(truncateDecimals(e.detail.axisMovementValue*97 * -1)) + "%";
        console.log(e.detail)
    
    }
}
/*
function getDriveMessage(ros){
    
    const cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: "command_control/ground_station_drive",
        messageType: "rover2_control_interface/msg/DriveCommandMessage"
      })
  
      const data = new ROSLIB.Message({
        
      })

      return data;
}
*/
function ChassisControl(props){
    
    
    //const message = getDriveMessage(props.ros)

    window.joypad.on('axis_move', function(e){slidingColor(e,props)});
    return(
        <article className="driveOutput" id = {props.id}>0%</article>
    );
}

export default ChassisControl;