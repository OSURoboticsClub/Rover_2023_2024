import React from 'react';
import UseRunOnce from './UseRunOnce.js'

function flipColor(e,id,button){
    
    if(e.detail.buttonName === button){
        
        var curColor = document.getElementById(id).style.backgroundColor;
        if(curColor === "red"){
            document.getElementById(id).style.backgroundColor = "green";
        } else {
            document.getElementById(id).style.backgroundColor = "red";
        }
    }
}
function Light(props){
    UseRunOnce({
        fn: () => {
            if(props.startColor){
                document.getElementById(props.id).style.backgroundColor = props.startColor;
            } else {
                document.getElementById(props.id).style.backgroundColor = "red";
            }
        }
    });
    
    if(props.controllerEvent && props.button)
        window.joypad.on(props.controllerEvent, function(e){flipColor(e,props.id,props.button)});
    
    
    return(
        <input className = "light" id = {props.id} value = {props.text} ></input>
    );
}

export default Light;