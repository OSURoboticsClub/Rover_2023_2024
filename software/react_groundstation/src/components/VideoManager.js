// import React,{useState,useEffect} from 'react';
// import ROSLIB from 'roslib';
// import Light from './LightControl.js';
// import axios from 'axios';
// import { DRIVE_CONTROLLER_ID } from '../lib/constants.js';

// function VideoManager(props){

//   let [towerImage,updateTowerImage] = useState("")
//   let [chassisImage,updateChassisImage] = useState("")

  
  
//   let ros = props.ros

//   var chassis_controller = new ROSLIB.Topic({
//     ros : ros,
//     name : '/cameras/chassis/camera_control',
//     messageType : 'rover2_camera_interface/msg/CameraControlMessage'
//   });
//   var tower_controller = new ROSLIB.Topic({
//     ros : ros,
//     name : '/cameras/main_navigation/camera_control',
//     messageType : 'rover2_camera_interface/msg/CameraControlMessage'
//   });


  
//   let chassisSetup = new ROSLIB.Message({
//     enable_small_broadcast: true,
//     enable_medium_broadcast: false,
//     enable_large_broadcast: false
//   })
//   let towerSetup = new ROSLIB.Message({
//     enable_small_broadcast: true,
//     enable_medium_broadcast: false,
//     enable_large_broadcast: false
//   })
  

  
//   useEffect(()=>{
//     chassis_controller.publish(chassisSetup)
//     tower_controller.publish(towerSetup)

   


//   },[])
  
//   useEffect(()=>{
//     console.log(towerImage)
//   },[towerImage])


//   useEffect(() => {
//     /*
//     var chassis_listener = new ROSLIB.Topic({
//         ros : ros,
//         name : '/cameras/chassis/image_640x360/compressed',
//         messageType : 'sensor_msgs/CompressedImage'
//     });
    
//     chassis_listener.subscribe(function(message) {
//       updateChassisImage("data:image/png;base64," + message.data)
        
//     });
//     */
//     // var tower_listener = new ROSLIB.Topic({
//     //   ros : ros,
//     //   name : '/cameras/main_navigation/forward',
//     //   messageType : 'sensor_msgs/CompressedImage'
        
//     // });

//     // tower_listener.subscribe(function(message) {
//     //   updateTowerImage("data:image/png;base64," + message.data)
//     // });
//   }, [])
  

//   return(
//     <div>
        
//         <Light text = "Chassis Cam" controllerID = {DRIVE_CONTROLLER_ID} controllerEvent = "button_press" button = "button_8" startColor = "green"/>
//         <Light text = "Tower Cam" controllerID = {DRIVE_CONTROLLER_ID} controllerEvent = "button_press" button = "button_8"/>
//         <h1>Tower cam</h1>
//         <div><img src={towerImage} id="tower_image"/></div>
//         <h1>Chassis cam</h1>
//         <div><img src={chassisImage} id="chassis_image"/></div>
//     </div>
//   );
// }

// export default VideoManager;


// import React,{useState,useEffect} from 'react';
// import ROSLIB from 'roslib';
// import Light from './LightControl.js';
// import axios from 'axios';
// import { DRIVE_CONTROLLER_ID } from '../lib/constants.js';

// function VideoManager(props){
//   var buttonArr = [0,0,0,0,0,0,0,0,0,0,0]
//   var stickArr = [0,0,0,0,0,0,0,0]

//   //   let ros = props.ros

//   var chassis_controller = new ROSLIB.Topic({
//     ros : ros,
//     name : '/cameras/chassis/camera_control',
//     messageType : 'rover2_camera_interface/msg/CameraControlMessage'
//   });
//   var tower_controller = new ROSLIB.Topic({
//     ros : ros,
//     name : '/cameras/main_navigation/camera_control',
//     messageType : 'rover2_camera_interface/msg/CameraControlMessage'
//   });
//   var infrared_controller = new ROSLIB.Topic({
//     ros : ros,
//     name : '/cameras/infrared/camera_control',
//     messageType : 'rover2_camera_interface/msg/CameraControlMessage'
//   });
//   var gripper_controller = new ROSLIB.Topic({
//     ros : ros,
//     name : '/cameras/gripper/camera_control',
//     messageType : 'rover2_camera_interface/msg/CameraControlMessage'
//   });

  
//   useEffect(() => {
//     const dpadPressed = window.joypad.on('button_press', function(e){
//       if(e.detail.gamepad["id"] !== ARM_CONTROLLER_ID){
//         return ;
//       }
//       if(e.detail.buttonName == "button_12"){

//       }
//       if(e.detail.buttonName == "button_13"){

//       }
//       if(e.detail.buttonName == "button_14"){

//       }
//       if(e.detail.buttonName == "button_15"){

//       }
      
//     })
//   })
// }

// export default VideoManager;