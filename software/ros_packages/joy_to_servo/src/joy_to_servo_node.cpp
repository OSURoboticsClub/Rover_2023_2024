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

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

// We'll just set up parameters here
const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string EEF_FRAME_ID = "rover_arm_gripper";
const std::string BASE_FRAME_ID = "rover_arm_base_link";

struct ControllerMappings {

  std::map<std::string, int> AXIS_MAP;
  std::map<std::string, int> BUTTON_MAP;

  std::map<std::string, double> AXIS_DEFAULTS;
  std::map<std::string, double> BUTTON_DEFAULTS;

};

// To change controls or setup a new controller, all you should to do is change the above enums and the follow 2
// functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A TwistStamped message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                     std::unique_ptr<control_msgs::msg::JointJog>& joint,
                     bool& use_ik, const ControllerMappings& controllerMappings, float& slowdown)
{
  // Give joint jogging priority because it is only buttons
  // If any joint jog command is requested, we are only publishing joint commands
  if(buttons[controllerMappings.BUTTON_MAP.at("MENU")]){
    use_ik = false;
    
  }
  else if(buttons[controllerMappings.BUTTON_MAP.at("CHANGE_VIEW")]){
    use_ik = true;
  }


  if(use_ik){ //ik controls
    if (axes[controllerMappings.AXIS_MAP.at("D_PAD_Y")] || buttons[controllerMappings.BUTTON_MAP.at("LEFT_BUMPER")] || buttons[controllerMappings.BUTTON_MAP.at("RIGHT_BUMPER")])
    {
      // Map the D_PAD to the proximal joints
      // joint->joint_names.push_back("base_joint");
      // joint->velocities.push_back(axes[controllerMappings.AXIS_MAP.at("D_PAD_X")]);
      joint->joint_names.push_back("shoulder_joint");
      joint->velocities.push_back(axes[controllerMappings.AXIS_MAP.at("D_PAD_Y")] * slowdown);
      // joint->joint_names.push_back("wrist_roll_joint");
      // joint->velocities.push_back(axes[controllerMappings.AXIS_MAP.at("D_PAD_X")]);
      // Map the diamond to the distal joints
      return false;
    }

    // The bread and butter: map buttons to twist commands
    twist->twist.linear.y = axes[controllerMappings.AXIS_MAP.at("LEFT_STICK_Y")] * slowdown;
    twist->twist.linear.x = -1.0 * axes[controllerMappings.AXIS_MAP.at("LEFT_STICK_X")] * slowdown;

    double lin_y_right = -0.5 * (axes[controllerMappings.AXIS_MAP.at("RIGHT_TRIGGER")] - controllerMappings.AXIS_DEFAULTS.at("RIGHT_TRIGGER"));
    double lin_y_left = 0.5 * (axes[controllerMappings.AXIS_MAP.at("LEFT_TRIGGER")] - controllerMappings.AXIS_DEFAULTS.at("LEFT_TRIGGER"));
    twist->twist.linear.z = (lin_y_right + lin_y_left) * slowdown;

    //pitch
    twist->twist.angular.x = axes[controllerMappings.AXIS_MAP.at("RIGHT_STICK_Y")] * slowdown;
    //Yaw
    twist->twist.angular.y = axes[controllerMappings.AXIS_MAP.at("RIGHT_STICK_X")] * slowdown;
    // Roll
    twist->twist.angular.z = axes[controllerMappings.AXIS_MAP.at("D_PAD_X")] * slowdown;

    // double roll_positive = buttons[controllerMappings.BUTTON_MAP.at("RIGHT_BUMPER")];
    // double roll_negative = -1 * (buttons[xontrollerMappings.BUTTON_MAP.at("LEFT_BUMPER")]);
    // twist->twist.angular.z = roll_positive + roll_negative;

    return true;
  }
  else{ //joint by joint control
    joint->joint_names.push_back("rover_arm_base_joint");
    joint->velocities.push_back(axes[controllerMappings.AXIS_MAP.at("D_PAD_X")] * -1.0);
    joint->joint_names.push_back("rover_arm_shoulder_joint");
    joint->velocities.push_back(axes[controllerMappings.AXIS_MAP.at("D_PAD_Y")] * -1.0);
    joint->joint_names.push_back("rover_arm_elbow_pitch_joint");
    joint->velocities.push_back(axes[controllerMappings.AXIS_MAP.at("LEFT_STICK_Y")] * -1.0); //THIS JOINT IS BACKWARDS
    joint->joint_names.push_back("rover_arm_elbow_roll_joint");
    joint->velocities.push_back(axes[controllerMappings.AXIS_MAP.at("LEFT_STICK_X")] * -1.0);
    joint->joint_names.push_back("rover_arm_wrist_pitch_joint");
    joint->velocities.push_back(axes[controllerMappings.AXIS_MAP.at("RIGHT_STICK_Y")] * -1.0);
    joint->joint_names.push_back("rover_arm_wrist_roll_joint");
    joint->velocities.push_back(axes[controllerMappings.AXIS_MAP.at("RIGHT_STICK_X")]);

    return false;
  }
}

/** \brief // This should update the frame_to_publish_ as needed for changing command frame via controller
 * @param frame_name Set the command frame to this
 * @param buttons The vector of discrete controller button values
 */
void updateCmdFrame(std::string& frame_name, const std::vector<int>& buttons, const ControllerMappings& controllerMappings)
{
  if (buttons[controllerMappings.BUTTON_MAP.at("CHANGE_VIEW")] && frame_name == EEF_FRAME_ID)
    frame_name = EEF_FRAME_ID;
  else if (buttons[controllerMappings.BUTTON_MAP.at("MENU")] && frame_name == BASE_FRAME_ID)
    frame_name = EEF_FRAME_ID;
}

class JoyToServoNode : public rclcpp::Node
{
public:
  JoyToServoNode(const rclcpp::NodeOptions& options)
    : Node("joy_to_twist_publisher", options), frame_to_publish_(EEF_FRAME_ID)
  {
    // Declare and get the controller type parameter
    this->declare_parameter<std::string>("controller_type", "xbox");
    std::string controller_type = this->get_parameter("controller_type").as_string();

    RCLCPP_INFO(this->get_logger(), "Using controller type: %s", controller_type.c_str());

    // Initialize the mappings based on controller type
    initializeControllerMappings(controller_type);

    use_ik = true;
    slowdown = 1.0;
    // Setup pub/sub
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { return joyCB(msg); });

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
    collision_pub_ =
        this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", rclcpp::SystemDefaultsQoS());

    // Create a service client to start the ServoNode
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

  }

  ~JoyToServoNode() override
  {
    if (collision_pub_thread_.joinable())
      collision_pub_thread_.join();
  }

  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    // This call updates the frame for twist commands
    updateCmdFrame(frame_to_publish_, msg->buttons, controller_map);

    // Convert the joystick message to Twist or JointJog and publish
    if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg, use_ik, controller_map, slowdown))
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
      joint_msg->header.frame_id = "rover_arm_base_link";
      joint_pub_->publish(std::move(joint_msg));
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

  bool use_ik;

  float slowdown;

  std::string frame_to_publish_;

  std::thread collision_pub_thread_;

  ControllerMappings controller_map;

  void initializeControllerMappings(const std::string& controller_type)
  {
      if (controller_type == "xbox")
      {
          // Xbox Controller Mapping
          controller_map.AXIS_DEFAULTS = { { "LEFT_TRIGGER", 1.0 }, { "RIGHT_TRIGGER", 1.0 } };

          controller_map.AXIS_MAP = {
              { "LEFT_STICK_X", 0 }, { "LEFT_STICK_Y", 1 }, { "LEFT_TRIGGER", 2 },
              { "RIGHT_STICK_X", 3 }, { "RIGHT_STICK_Y", 4 }, { "RIGHT_TRIGGER", 5 },
              { "D_PAD_X", 6 }, { "D_PAD_Y", 7 }
          };

          controller_map.BUTTON_MAP = {
              { "A", 0 }, { "B", 1 }, { "X", 2 }, { "Y", 3 },
              { "LEFT_BUMPER", 4 }, { "RIGHT_BUMPER", 5 },
              { "CHANGE_VIEW", 6 }, { "MENU", 7 },
              { "HOME", 8 }, { "LEFT_STICK_CLICK", 9 }, { "RIGHT_STICK_CLICK", 10 }
          };
      }
      else if (controller_type == "ps")
      {
          // PlayStation Controller Mapping
          controller_map.AXIS_DEFAULTS = { { "LEFT_TRIGGER", 1.0 }, { "RIGHT_TRIGGER", 1.0 } };

          controller_map.AXIS_MAP = {
              { "LEFT_STICK_X", 0 }, { "LEFT_STICK_Y", 1 }, { "LEFT_TRIGGER", 2 },
              { "RIGHT_STICK_X", 3 }, { "RIGHT_STICK_Y", 4 }, { "RIGHT_TRIGGER", 5 },
              { "D_PAD_X", 6 }, { "D_PAD_Y", 7 }
          };

          controller_map.BUTTON_MAP = {
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
};  // class JoyToServoNode

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::spin(std::make_shared<JoyToServoNode>(node_options));
  rclcpp::shutdown();
  return 0;
}
