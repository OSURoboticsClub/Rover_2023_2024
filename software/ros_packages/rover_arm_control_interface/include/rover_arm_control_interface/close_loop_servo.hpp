#ifndef CLOSE_LOOP_SERVO_HPP
#define CLOSE_LOOP_SERVO_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/exceptions.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <array>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <std_srvs/srv/trigger.hpp>

class CloseLoopServo : public rclcpp::Node, public std::enable_shared_from_this<CloseLoopServo>
{
public:
    CloseLoopServo();

private:
    // Moveit msg Type aliases for readability
    using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

    // Robot State
    geometry_msgs::msg::PoseStamped current_pose_; 
    geometry_msgs::msg::PoseStamped target_pose_;
    geometry_msgs::msg::TwistStamped controller_twist_;

    float kp_ = 0.5; // Proportional gain for servoing
    float ki_ = 0.0; // Integral gain for servoing
    float kd_ = 0.1; // Derivative gain for servoing

    bool run_close_loop_servo_ = false; // Flag to control servoing loop
    bool reset_position_ = true; // Flag to trigger position reset

    // TF2 components
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    //Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_servo_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_servo_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_position_srv_;

    //Subscribers
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr joy_twist_sub_;

    //Publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_twist_pub_;

    //Timers
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback();

    void joy_twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    void stop_servo_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void start_servo_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void reset_position_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    geometry_msgs::msg::TwistStamped zero_twist();

};

#endif  // CLOSE_LOOP_SERVO_HPP_