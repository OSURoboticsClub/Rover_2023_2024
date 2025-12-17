/*********************************************************************
  * Software License Agreement (BSD License)
  *
  *  Copyright (c) 2020, PickNik Inc.
  *  All rights reserved.
  *
  *  Redistribution and use in source and binary forms, with or without
  *  modification, are permitted provided that the following conditions
  *  are met:
  *
  *   * Redistributions of source code must retain the above copyright
  *     notice, this list of conditions and the following disclaimer.
  *   * Redistributions in binary form must reproduce the above
  *     copyright notice, this list of conditions and the following
  *     disclaimer in the documentation and/or other materials provided
  *     with the distribution.
  *   * Neither the name of PickNik Inc. nor the names of its
  *     contributors may be used to endorse or promote products derived
  *     from this software without specific prior written permission.
  *
  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  *  POSSIBILITY OF SUCH DAMAGE.
  *********************************************************************/
  
 /*      Title     : joystick_servo_example.cpp
  *      Project   : moveit_servo
  *      Created   : 08/07/2020
  *      Author    : Adam Pettinger
  */

/*
Joy to Servo Node
DAM Robotics
Authors: Jared Northrop
Year: 2526

The follow is based off the above work by Adam Pettinger at PickNik Robotics for MoveIt Servo. It is modified
to fit the needs of rover tele-op control. This node takes a joy message and converts it to a form capatable 
with MoveIt Servo. 

Notes
-----
- This node currently runs in unitless mode of moveit servo. This should be changed to acutal units in which 
 the node should take in the joint_limits.yaml to find velocity limits for each joint and the kinematics.yaml
 to find cartesion limits. 

Licenses
--------
The below code is covered under the 3-clause BSD license.
*/


#include "rover_arm_control_interface/joy_to_servo.hpp"

/*Converts a joy message to a TwistStamped or JointJog message

Parameters
----------
axes : vector<float>
    The axes from the joy message
buttons : vector<int>
    The buttons from the joy message  
use_ik : bool
    Whether to use inverse kinematics or joint jogging
controllerMappings : ControllerMappings
    The controller mappings struct
slowdown : float
    Scalar to slow down velocity

Returns
-------
twist : unique_ptr<TwistStamped>
    The TwistStamped message to populate
joint : unique_ptr<JointJog>
    The JointJog message to populate  
bool
    True if publishing TwistStamped, false if publishing JointJog

Notes
-----
- Need to switch output message to be in actual units (m/s, rad/s) rather than normalized -1 to 1

- Lights and laser should be in the gripper node not here anyways, but it is incomplete. 
*/
bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                     std::unique_ptr<control_msgs::msg::JointJog>& joint,
                     bool& use_ik, const ControllerMappings& controllerMappings, float& slowdown)
{

  if (buttons[controllerMappings.BUTTON_MAP.at("RIGHT_BUMPER")]){
    //Turn on/off lights
    slowdown = 1.0;
  }
  if (buttons[controllerMappings.BUTTON_MAP.at("LEFT_BUMPER")]){
    //Turn on/off laser
    slowdown = 1.0;
  }
  if(use_ik){ //ik controls
    //Maintain joint by joint shoulder control if D_PAD is used
    if (axes[controllerMappings.AXIS_MAP.at("D_PAD_Y")])
    {
      joint->joint_names.push_back("shoulder_joint");
      joint->velocities.push_back(axes[controllerMappings.AXIS_MAP.at("D_PAD_Y")] * slowdown);
      return false;
    }

    //Map joystick to twist commands
    twist->twist.linear.y = axes[controllerMappings.AXIS_MAP.at("LEFT_STICK_Y")] * slowdown;
    twist->twist.linear.x = -1.0 * axes[controllerMappings.AXIS_MAP.at("LEFT_STICK_X")] * slowdown;

    double lin_z_in = -1.0 * (axes[controllerMappings.AXIS_MAP.at("RIGHT_TRIGGER")] - controllerMappings.AXIS_DEFAULTS.at("RIGHT_TRIGGER"));
    double lin_y_out = 1.0 * (axes[controllerMappings.AXIS_MAP.at("LEFT_TRIGGER")] - controllerMappings.AXIS_DEFAULTS.at("LEFT_TRIGGER"));
    twist->twist.linear.z = (lin_y_out + lin_z_in) * slowdown;

    //pitch
    twist->twist.angular.x = axes[controllerMappings.AXIS_MAP.at("RIGHT_STICK_Y")] * slowdown;
    //Yaw
    twist->twist.angular.y = -1.0 * axes[controllerMappings.AXIS_MAP.at("RIGHT_STICK_X")] * slowdown;
    // Roll
    twist->twist.angular.z = axes[controllerMappings.AXIS_MAP.at("D_PAD_X")] * slowdown;

    return true;
  }
  else{ //joint by joint control
    joint->joint_names.push_back("base_joint");
    joint->velocities.push_back(axes[controllerMappings.AXIS_MAP.at("D_PAD_X")] * -1.0);
    joint->joint_names.push_back("shoulder_joint");
    joint->velocities.push_back(axes[controllerMappings.AXIS_MAP.at("D_PAD_Y")] * -1.0);
    joint->joint_names.push_back("elbow_pitch_joint");
    joint->velocities.push_back(axes[controllerMappings.AXIS_MAP.at("LEFT_STICK_Y")] * 1.0);
    joint->joint_names.push_back("elbow_roll_joint");
    joint->velocities.push_back(axes[controllerMappings.AXIS_MAP.at("LEFT_STICK_X")] * -1.0);
    joint->joint_names.push_back("wrist_pitch_joint");
    joint->velocities.push_back(axes[controllerMappings.AXIS_MAP.at("RIGHT_STICK_Y")] * -1.0);
    joint->joint_names.push_back("wrist_roll_joint");
    joint->velocities.push_back(axes[controllerMappings.AXIS_MAP.at("RIGHT_STICK_X")]);

    return false;
  }

}

/*This function updates the command frame for servo commands. Switches between the EE frame and base frame. 

Parameters
----------
buttons : vector<int>
    The buttons pressed on the joystick
controllerMappings : ControllerMappings
    The controller mappings struct

Returns
-------
frame_name : string
    The frame to publish
use_ik : bool
    Whether to use inverse kinematics or joint jogging

Notes
-----
- Need to split changing frames and changing control mode. 
*/
void updateCmdFrame(std::string& frame_name, const std::vector<int>& buttons, const ControllerMappings& controllerMappings, bool& use_ik)
{
  if (buttons[controllerMappings.BUTTON_MAP.at("CHANGE_VIEW")] && frame_name == EEF_FRAME_ID){
    frame_name = BASE_FRAME_ID;
    use_ik = false; 
  }
  else if (buttons[controllerMappings.BUTTON_MAP.at("MENU")] && frame_name == BASE_FRAME_ID){
    frame_name = EEF_FRAME_ID;
    use_ik = true;
  }
}

namespace moveit_servo
{

JoyToServoPub::JoyToServoPub(const rclcpp::NodeOptions& options)
    : Node("joy_to_servo_pub", options), use_ik_(true), slowdown_(1.0)
{
    // Ros Node Params
    this->declare_parameter<std::string>("controller_type", "xbox");
    std::string controller_type = this->get_parameter("controller_type").as_string();

    //Setup Subscribers
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { return joyCB(msg); });


    //Setup Publishers
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS()); //delta_twist_cmds
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS()); //delta_joint_cmds
    collision_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", rclcpp::SystemDefaultsQoS()); //planning_scene
  

    //Setup Services
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    //Initialize mappings
    RCLCPP_INFO(this->get_logger(), "Using controller type: %s", controller_type.c_str());
    this->initializeControllerMappings(controller_type);

}

JoyToServoPub::~JoyToServoPub()
{
    if (collision_pub_thread_.joinable())
    collision_pub_thread_.join();
}


/*Initialize controller mappings for either ps or a xbox controller
Parameters
----------
controller_type : "ps", "xbox"
    The type of controller being used.
*/
void JoyToServoPub::initializeControllerMappings(const std::string& controller_type)
  {
      if (controller_type == "xbox")
      {
          // Xbox Controller Mapping
          controller_map_.AXIS_DEFAULTS = { { "LEFT_TRIGGER", 1.0 }, { "RIGHT_TRIGGER", 1.0 } };

          controller_map_.AXIS_MAP = {
              { "LEFT_STICK_X", 0 }, { "LEFT_STICK_Y", 1 }, { "LEFT_TRIGGER", 2 },
              { "RIGHT_STICK_X", 3 }, { "RIGHT_STICK_Y", 4 }, { "RIGHT_TRIGGER", 5 },
              { "D_PAD_X", 6 }, { "D_PAD_Y", 7 }
          };

          controller_map_.BUTTON_MAP = {
              { "A", 0 }, { "B", 1 }, { "X", 2 }, { "Y", 3 },
              { "LEFT_BUMPER", 4 }, { "RIGHT_BUMPER", 5 },
              { "CHANGE_VIEW", 6 }, { "MENU", 7 },
              { "HOME", 8 }, { "LEFT_STICK_CLICK", 9 }, { "RIGHT_STICK_CLICK", 10 }
          };
      }
      else if (controller_type == "ps")
      {
          // PlayStation Controller Mapping
          controller_map_.AXIS_DEFAULTS = { { "LEFT_TRIGGER", 1.0 }, { "RIGHT_TRIGGER", 1.0 } };

          controller_map_.AXIS_MAP = {
              { "LEFT_STICK_X", 0 }, { "LEFT_STICK_Y", 1 }, { "LEFT_TRIGGER", 2 },
              { "RIGHT_STICK_X", 3 }, { "RIGHT_STICK_Y", 4 }, { "RIGHT_TRIGGER", 5 },
              { "D_PAD_X", 6 }, { "D_PAD_Y", 7 }
          };

          controller_map_.BUTTON_MAP = {
              { "A", 0 }, { "B", 1 }, { "X", 2 }, { "Y", 3 }, // X, CIRCLE, TRIANGLE, SQUARE
              { "LEFT_BUMPER", 4 }, { "RIGHT_BUMPER", 5 },
              {"LEFT_TRIGGER", 6}, {"RIGHT_TRIGGER", 7},
              { "CHANGE_VIEW", 8 }, { "MENU", 9 }, //SHARE, OPTIONS
              { "HOME", 10 }, { "LEFT_STICK_CLICK", 11 }, { "RIGHT_STICK_CLICK", 12}
          };
      }
      else
      {
          RCLCPP_WARN(this->get_logger(), "Unknown controller type. Defaulting to Xbox mapping.");
          initializeControllerMappings("xbox");  // Default to Xbox
      }
  }


void JoyToServoPub::joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    // This call updates the frame for twist commands
    updateCmdFrame(frame_to_publish_, msg->buttons, controller_map_, use_ik_);

    // Convert the joystick message to Twist or JointJog and publish
    if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg, use_ik_, controller_map_, slowdown_))
    {
      // publish the TwistStamped
      twist_msg->header.frame_id = frame_to_publish_;
      twist_msg->header.stamp = this->now();
      twist_pub_->publish(std::move(twist_msg));
    }
    else
    {
      // publish the JointJog
      joint_msg->header.stamp = this->now();
      //Likely doesn't matter what frame
      joint_msg->header.frame_id = BASE_FRAME_ID;
      joint_pub_->publish(std::move(joint_msg));
    }
  }
} // namespace moveit_servo

 // Register the component with class_loader
 #include <rclcpp_components/register_node_macro.hpp>
 RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::JoyToServoPub)