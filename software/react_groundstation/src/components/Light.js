import React,{useState} from 'react';
import UseRunOnce from './UseRunOnce.js'

function flipColor(e,button,backgroundColor,changeColor){
    
    if(e.detail.buttonName === button){
          
        if(backgroundColor === "red"){
            changeColor("green");
        } else {
            changeColor("red");
        }
    }
}
function Light(props){
    let [backgroundColor,changeColor] = useState("red")
    UseRunOnce({
        fn: () => {
            if(props.startColor){
                changeColor(props.startColor);
            } else {
                changeColor("red");
            }
        }
    });
    
    if(props.controllerEvent && props.button)
        window.joypad.on(props.controllerEvent, function(e){flipColor(e,props.button,backgroundColor,changeColor)});
    
    
    return(
        <input className = "light" id = {props.id} value = {props.text} style={{backgroundColor: backgroundColor}}></input>
    );
}

export default Light;