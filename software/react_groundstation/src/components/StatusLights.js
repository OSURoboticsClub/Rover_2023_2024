
import Light from './LightControl.js';
import React,{useState,useEffect} from 'react';
import ROSLIB from 'roslib';
import haversine from 'haversine'
function StatusLights(props){
    const [latitude,setLatitude] = useState(0)
    const [longitude, setLongitude] = useState(0)
    const [targetLatitude,setTargetLatitude] = useState(0)
    const [targetLongitude, setTargetLongitude] = useState(0)
    const [prevLatitude,setPrevLatitude] = useState(0)
    const [prevLongitude, setPrevLongitude] = useState(0)
    const [targetAngle, setTargetAngle] = useState(0)
    const [haversineDistance, setHaversineDistance] = useState(0)
    const gpsTopic = new ROSLIB.Topic({
        ros: props.ros,
        name: "/tower/status/gps",
        messageType: "rover2_control_interface/msg/GPSStatusMessage"
    })
    const handleLongChange = (event) => {
        const value = event.target.value;
        setPrevLongitude(longitude)
        setTargetLongitude(parseFloat(value));
      };
    const handleLatChange = (event) => {
      const value = event.target.value;
      setPrevLatitude(latitude)
      setTargetLatitude(parseFloat(value));
    };
    useEffect(()=>{
        gpsTopic.subscribe(function(message) {
            setLatitude(message.rover_latitude)
            setLongitude(message.rover_longitude)
            
        })
    },[])

    useEffect(()=>{
        setHaversineDistance(haversine({"latitude":latitude,"longitude":longitude},{"latitude":targetLatitude,"longitude":targetLongitude},{unit: 'meter'}))
        setTargetAngle(Math.atan((latitude-prevLatitude)/(longitude-prevLongitude)) * (180/Math.PI))
            
    },[latitude,longitude,targetLatitude,targetLongitude])
    //window.joypad.on('button_press', function(){flipColor("square1")});
    return (
        <div>
            <p>
                <Light text = "Chassis Cam" id = "chas_cam" controllerEvent = "button_press" button = "button_8" startColor = "green"/>
                <Light text = "Tower Cam" id = "tower_cam" controllerEvent = "button_press" button = "button_8"/>
                
            </p>
            <h2>
             Current latitude: {latitude} &emsp; Target latitude: 
                <input type ="number" onChange ={handleLatChange} value = {targetLatitude} /> 
            </h2>
            
            <h2>
             Current longitude: {longitude} &emsp; Target longitude: 
                <input type ="number" onChange ={handleLongChange} value = {targetLongitude}/> 
            </h2>
            <h2>Distance from target: {haversineDistance} Angle to target: {targetAngle}</h2>
            
        </div>
    );
}

export default StatusLights;