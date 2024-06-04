
import '../lib/controller.js';

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
    //LIST TOPICS RECEIVED BY ROSLIB
    
    var topicsClient = new ROSLIB.Service({
    ros : ros,
    name : '/rosapi/topics',
    serviceType : 'rosapi/Topics'
    });
  
    var request = new ROSLIB.ServiceRequest();
  
    topicsClient.callService(request, function(result) {
    console.log("Getting topics...");
    console.log(result.topics);
    });
    
    window.joypad.on('axis_move', (e) => {
      console.log(e.detail);
      
    });
   
    window.joypad.on('button_held', (e) => {
        const { buttonName } = e.detail;
      
        console.log(`${buttonName} was held!`);
      });
    
    return (
        
        <div className = "main">
            <div className = "leftScreen">
                <div className = "videoManager">
                    <VideoManager ros = {ros}/>
                </div>
            </div>
            <div className = "rightScreen">
                <div className = "status">
                    <StatusLights ros = {ros}/>
                </div>
                <div className = "mining">    
                </div>
                <div className = "control">
                    <ChassisManager ros = {ros}/>
                </div>
            </div>
        </div>
        
        
    );

}



export default MainPage;