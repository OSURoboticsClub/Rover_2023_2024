/*
Close Loop Servo Node
DAM Robotics
Authors: Jared Northrop
Year: 2526

This node is to correct arm drift during Ik operations using Moveit Servo. 

Notes
This Node is not operational and is only in the early stages of development. No actual logic has been devised
or created for the implementation. This is a complex task requiring looking at the manipulability elipsoid, 
and the configuration of the robot to ensure paths do not violate velocity or acceleration constraints.
*/

#include "rover_arm_control_interface/close_loop_servo.hpp"

#include <geometry_msgs/msg/pose.hpp>

CloseLoopServo::CloseLoopServo()
: Node("closed_loop_servo")
{

    // Get path parameter
    this->declare_parameter<std::string>("seam_file_path", "/home/jn2/college/Cobot_Welding_Masters_Project/data/1001.dat");
    this->get_parameter("seam_file_path", this->path_);
    
    //Services
    // cartesian_client_ =
    //     this->create_client<CartesianPath>("/compute_cartesian_path");
    // Wait for services
    // while (!cartesian_client_->wait_for_service(std::chrono::seconds(1))) {
    //     RCLCPP_INFO(this->get_logger(), "Waiting for cartesian service...");
    // }
    start_servo_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "start_close_loop_servo",
        std::bind(&CloseLoopServo::start_servo_callback, this, std::placeholders::_1, std::placeholders::_2)
    )

    stop_servo_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "stop_close_loop_servo",
        std::bind(&CloseLoopServo::stop_servo_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    reset_position_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "reset_servo_position",
        std::bind(&CloseLoopServo::reset_position_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    //Actions


    //Subscriptions
    joy_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "joy_twist", 
        10, 
        std::bind(&CloseLoopServo::joy_twist_callback, this, std::placeholders::_1)
    );

    servo_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("servo_twist", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), 
      std::bind(&CloseLoopServo::timer_callback, 
      this
    ));


    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}

void ClosedLoopServo::timer_callback(){
    if (this->run_close_loop_servo_) {
        if this->reset_position_ {
            // Get current pose and set as target
            try {
                auto transform = tf_buffer_->lookupTransform("rover_arm_base_link", "rover_arm_tool0", tf2::TimePointZero);
                current_pose_.header.stamp = this->get_clock()->now();
                current_pose_.header.frame_id = "rover_arm_base_link";
                current_pose_.pose.position.x = transform.transform.translation.x;
                current_pose_.pose.position.y = transform.transform.translation.y;
                current_pose_.pose.position.z = transform.transform.translation.z;
                current_pose_.pose.orientation = transform.transform.rotation;

                this->target_pose_ = this->current_pose_;
                RCLCPP_INFO(this->get_logger(), "Position reset. Current pose set as target.");
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
                RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                  toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            }
            this->reset_position_ = false;
        }
      //Update target state based on controller 
        
      //Calculate error between current and target

      //Apply PID to get corrected twist command

    }
}

void ClosedLoopServo::joy_twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
    this->controller_twist_ = *msg;
}

geometry_msgs::msg::TwistStamped ClosedLoopServo::zero_twist(){
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = this->get_clock()->now();
    twist.header.frame_id = "rover_arm_base_link";
    twist.twist.linear.x = 0.0;
    twist.twist.linear.y = 0.0;
    twist.twist.linear.z = 0.0;
    twist.twist.angular.x = 0.0;
    twist.twist.angular.y = 0.0;
    twist.twist.angular.z = 0.0;
    return twist;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CloseLoopServo>();
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ur_manipulator");
    node->set_move_group(move_group);
    // node->cartesian_path();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}