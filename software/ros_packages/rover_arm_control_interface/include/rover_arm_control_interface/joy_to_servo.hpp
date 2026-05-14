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

Notes:
------
- This node currently runs in unitless mode of moveit servo. This should be changed to acutal units in which 
 the node should take in the joint_limits.yaml to find velocity limits for each joint and the kinematics.yaml
 to find cartesion limits. 

Licenses:
---------
The below code is covered under the 3-clause BSD license.
*/

#ifndef JOY_TO_SERVO_HPP
#define JOY_TO_SERVO_HPP
 
 #include <sensor_msgs/msg/joy.hpp>
 #include <geometry_msgs/msg/twist_stamped.hpp>
 #include <control_msgs/msg/joint_jog.hpp>
 #include <std_srvs/srv/trigger.hpp>
 #include <moveit_msgs/msg/planning_scene.hpp>
 #include <rclcpp/client.hpp>
 #include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
 #include <rclcpp/node.hpp>
 #include <rclcpp/publisher.hpp>
 #include <rclcpp/qos.hpp>
 #include <rclcpp/qos_event.hpp>
 #include <rclcpp/subscription.hpp>
 #include <rclcpp/time.hpp>
 #include <rclcpp/utilities.hpp>
 #include <thread>


const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string EEF_FRAME_ID = "rover_arm_tool0";
const std::string BASE_FRAME_ID = "rover_arm_base_link";

struct ControllerMappings {

  std::map<std::string, int> AXIS_MAP;
  std::map<std::string, int> BUTTON_MAP;

  std::map<std::string, double> AXIS_DEFAULTS;
  std::map<std::string, double> BUTTON_DEFAULTS;

};

bool convertJoyToCmd(
    const std::vector<float>& axes,
    const std::vector<int>& buttons,
    std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
    std::unique_ptr<control_msgs::msg::JointJog>& joint);

void updateCmdFrame(
    std::string& frame_name,
    const std::vector<int>& buttons);

namespace moveit_servo
{
class JoyToServoPub : public rclcpp::Node
{
public:
    explicit JoyToServoPub(const rclcpp::NodeOptions& options);
    ~JoyToServoPub() override;

private:
    auto joint_params = this->list_parameters({"joint_limits"}, 3)
    void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

    std::string frame_to_publish_{EEF_FRAME_ID};
    std::thread collision_pub_thread_;
    ControllerMappings controller_map_;
    bool use_ik_{true}; //Whether to use inverse kinematics or joint jogging
    float slowdown_{1.0}; //Scalar to slow down velocity


  void initializeControllerMappings(const std::string& controller_type);
};
} // namespace moveit_servo

#endif //JOY_TO_SERVO_HPP
