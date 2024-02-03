import React from 'react';
import '../lib/controller.js';
//import '../components/statusBox.js'
//import '../components/sliderBox.js'
//import '../components/streamBox.js'
import StatusLights from '../components/StatusLights.js'
import DriveInfo from '../components/DriveInfo.js'




function SlidingColor(e,squareName){

        var newColor = 125+ e.detail.axisMovementValue*100;

        document.getElementById(squareName).style.backgroundColor = "rgb("+newColor+","+newColor+","+newColor+")";
        console.log("rgb("+newColor+","+newColor+","+newColor+")");
}

function MainPage(){
    
    //window.joypad.on('button_press', function(){ChangeColor("square1")});
    //window.joypad.on('button_held', function(){ChangeColor("square2")});
    //window.joypad.on('axis_move', function(e){SlidingColor(e,"square3")});
    
    /*
    <div id = "square2"/>
    <div id = "square3"/>
    <input id = "square1" value = "Bogie"  showonly></input>
    <input id = "square1" value = "Bogie"  showonly></input>


    <div class = "mining">
                </div>
                <div class = "control">
                </div>
    */
    return (
        
        <div className = "main">
            <div className = "leftScreen">
                <div className = "status">
                    <StatusLights/>
                </div>
                <div className = "mining">    
                </div>
                <div className = "control">
                    <DriveInfo/>
                </div>
            </div>
            <div className = "rightScreen">
                <div className = "streamLarge">
                </div>
                <div className = "streamSmall">
                </div>
            </div>
        </div>
        
        
    );

}



export default MainPage;