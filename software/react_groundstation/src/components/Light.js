import React from 'react';


function ChangeColor(id){
    var curColor = document.getElementById(id).style.backgroundColor;
    if(curColor === "red"){
        document.getElementById(id).style.backgroundColor = "green";
    } else {
        document.getElementById(id).style.backgroundColor = "red";
    }
    
}
function Light(props){
    
    return(
        <input className = "light" id = {props.id} value = {props.text} ></input>
    );
}

export default Light;