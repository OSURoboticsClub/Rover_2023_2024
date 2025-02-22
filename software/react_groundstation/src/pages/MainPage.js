
import '../lib/controller.js';

import StatusLights from '../components/StatusLights.js'
import ChassisManager from '../components/ChassisManager.js'
import VideoManager from '../components/VideoManager.js'
import FrontMountManager from '../components/FrontMountManager.js'
import CameraPanTilt from '../components/CameraPanTilt.js';

import ROSLIB from 'roslib'

function MainPage(){
    var ethIP = 'ws://192.168.1.139:9090'
    var tpLinkIP = 'ws://192.168.1.101:9090'
    var connectEth = true
    
    var ros = new ROSLIB.Ros({
      url : tpLinkIP
    });

    ros.on('connection', function() {
      console.log('Connected to websocket server.\n\n\n\n\n\n\n\n');
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
      //console.log(e.detail);
      
    });
   

    
    return (
        
        <div className = "main">
            {/* <div className = "leftScreen">
                <div className = "videoManager">
                    <VideoManager ros = {ros}/>
                </div>
            </div> */}
            <div className = "rightScreen">
                <div className = "status">
                    <StatusLights ros = {ros}/>
                    <CameraPanTilt  ros = {ros} button_switch = "button_8" button_center = "button_9" buttons_move = {["button_2",//"button_12"
                                                                                                                                                 "button_0",//"button_13"
                                                                                                                                                  "button_1",//"button_14*"
                                                                                                                                                  "button_3"]}/*"button_15"*//>
                </div>
                <div className = "mining"> 
                    <FrontMountManager ros = {ros}/>
                </div>
                <div className = "control">
                    <ChassisManager ros = {ros}/>
                    <VideoManager ros = {ros}/>
                </div>
            </div>
        </div>
        
        
    );

}



export default MainPage;