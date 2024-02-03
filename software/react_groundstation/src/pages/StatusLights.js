/*
import React from 'react';
import '../lib/controller.js';

const colors = ["blue", "red", "green", "yellow", "orange", "cyan", "purple", "teal", "brown"];

function ChangeColor(squareName){
    
    const chooser = colors[Math.floor(Math.random() * colors.length)];
    document.getElementById(squareName).style.backgroundColor = chooser;        
    
}

function SlidingColor(e,squareName){

        var newColor = 125+ e.detail.axisMovementValue*100;

        document.getElementById(squareName).style.backgroundColor = "rgb("+newColor+","+newColor+","+newColor+")";
        console.log("rgb("+newColor+","+newColor+","+newColor+")");
}

function StatusLights(){
    
    window.joypad.on('button_press', function(){ChangeColor("square1")});
    window.joypad.on('button_held', function(){ChangeColor("square2")});
    window.joypad.on('axis_move', function(e){SlidingColor(e,"square3")});
    
   
    return (
        
        <article>
            <div id="square1"/>
            <div id="square2"/>
            <div id="square3"/>
            
        </article>
        
        
    );

}



export default StatusLights;
*/