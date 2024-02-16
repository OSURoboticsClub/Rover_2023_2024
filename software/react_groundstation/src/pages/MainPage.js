import React from 'react';
import '../lib/controller.js';
//import '../components/statusBox.js'
//import '../components/sliderBox.js'
//import '../components/streamBox.js'
import StatusLights from '../components/StatusLights.js'
import ChassisManager from '../components/ChassisManager.js'
import VideoManager from '../components/VideoManager.js'

import ROSLIB from 'roslib'

function MainPage(){
    var ros = new ROSLIB.Ros({
      url : 'ws://192.168.1.138:9090'
    });

    ros.on('connection', function() {
      console.log('Connected to websocket server.');
    });

    ros.on('error', function(error) {
      console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', function() {
      console.log('Connection to websocket server closed.');
    });



    
    return (
        
        <div className = "main">
            <div className = "leftScreen">
                <div className = "status">
                    <StatusLights ros = {ros}/>
                </div>
                <div className = "mining">    
                </div>
                <div className = "control">
                    <ChassisManager ros = {ros}/>
                </div>
            </div>
            <div className = "rightScreen">
                <div className = "videoManager">
                    <VideoManager ros = {ros}/>
                </div>
            </div>
        </div>
        
        
    );

}



export default MainPage;