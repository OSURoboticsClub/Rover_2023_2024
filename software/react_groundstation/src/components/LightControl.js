import {useState,useEffect} from 'react';
//import UseRunOnce from './UseRunOnce.js'


function Light(props){
    let startColor = ''
    if(props.startColor){
        startColor = props.startColor;
    } else {
        startColor = "red"
    }
    let [backgroundColor,changeColor] = useState(startColor)
    
    const flipColor = (e) => {
        if(e.detail.buttonName === props.button){
            
            if(backgroundColor === "red"){    
                
                changeColor("green");
                
            } else {
                changeColor("red");
                
            }
        }
    }
    
    if(props.controllerEvent && props.button){
       window.joypad.on(props.controllerEvent, function(e){flipColor(e)});
    }
        
        
 
    return(
        <input className = "light" id = {props.id} value = {props.text} style={{backgroundColor: backgroundColor}} readOnly></input>
    );
}

export default Light;