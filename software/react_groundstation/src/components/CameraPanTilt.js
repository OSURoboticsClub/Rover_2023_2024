import React from 'react';
import ROSLIB from 'roslib'

const DEFAULT_TOWER_PAN_TILT_COMMAND_TOPIC = "tower/pan_tilt/control"
const DEFAULT_CHASSIS_PAN_TILT_COMMAND_TOPIC = "chassis/pan_tilt/control"

const TOWER_PAN_TILT_X_AXIS_SCALAR = 2
const TOWER_PAN_TILT_Y_AXIS_SCALAR = 15

const CHASSIS_PAN_TILT_X_AXIS_SCALAR = 100
const CHASSIS_PAN_TILT_Y_AXIS_SCALAR = 100

var currentTopic = DEFAULT_CHASSIS_PAN_TILT_COMMAND_TOPIC
var currentXScalar = CHASSIS_PAN_TILT_X_AXIS_SCALAR
var currentYScalar = CHASSIS_PAN_TILT_Y_AXIS_SCALAR

var rosTopic = null

function buttonPressHandler(e,props,topic){
    if(e.detail.buttonName === props.button_switch) {
        switchControlTopic(props)
    } else if(props.buttons_move.includes(e.detail.buttonName)){
        panTilt(e,props,topic)
    }else if(e.detail.buttonName === props.button_center){
        publishMessage(true,0,0,topic)
    }
}

function switchControlTopic(props){
    
    if(currentTopic === DEFAULT_TOWER_PAN_TILT_COMMAND_TOPIC){
        currentTopic = DEFAULT_CHASSIS_PAN_TILT_COMMAND_TOPIC;
        currentXScalar = CHASSIS_PAN_TILT_X_AXIS_SCALAR
        currentYScalar = CHASSIS_PAN_TILT_Y_AXIS_SCALAR

    } else {
        currentTopic = DEFAULT_TOWER_PAN_TILT_COMMAND_TOPIC;
        currentXScalar = TOWER_PAN_TILT_X_AXIS_SCALAR
        currentYScalar = TOWER_PAN_TILT_Y_AXIS_SCALAR
    }
    rosTopic = new ROSLIB.Topic({
        ros: props.ros,
        name: currentTopic,
        messageType: "rover2_control_interface/msg/TowerPanTiltControlMessage"
    })
    console.log(rosTopic)
    
    

}
//{button_12,button_13,button_14,button_15}
//up -12 0
//down -13 1
//left -14 2
//right -15 3
function panTilt(e,props,topic){
    var change_x = 0
    var change_y = 0
    if(e.detail.buttonName === props.buttons_move[0]){
        change_y = currentYScalar
    }
    if(e.detail.buttonName === props.buttons_move[1]){
        change_y = -currentYScalar
    }
    if(e.detail.buttonName === props.buttons_move[2]){
        change_x = currentXScalar
    }
    if(e.detail.buttonName === props.buttons_move[3]){
        change_x = -currentXScalar
    }
    
    publishMessage(false,change_x,change_y,topic)

}



function publishMessage(shouldCenter, relativePanAdjustment,relativeTiltAdjustment,topic){
    var data = new ROSLIB.Message({
        should_center: shouldCenter,
        relative_pan_adjustment: relativePanAdjustment,
        relative_tilt_adjustment: relativeTiltAdjustment,
        hitch_servo_positive: false,
        hitch_servo_negative: false
    });
    console.log('msg', data)
    topic.publish(data);
}


//button_switch
//button_center
//buttons_move

function CameraPanTilt(props){

    rosTopic = new ROSLIB.Topic({
      ros: props.ros,
      name: currentTopic,
      messageType: "rover2_control_interface/msg/TowerPanTiltControlMessage"
    })

    window.joypad.on('button_press', function(e){buttonPressHandler(e,props,rosTopic)});
}
export default CameraPanTilt;