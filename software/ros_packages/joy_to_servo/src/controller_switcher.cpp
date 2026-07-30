#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class ControllerSwitcher : public rclcpp::Node
{
public:
  ControllerSwitcher()
  : Node("controller_switcher")
  {
    // Parameters
    this->declare_parameter("home_button_index", 8);
    this->declare_parameter("controllers_to_toggle", std::vector<std::string>{"rover_arm_velocity_controller", "rover_arm_controller_moveit"});
    // Switching to this controller must call /servo_node/start_servo first, or moveit_servo's
    // low-pass filters/internal state can be stale relative to wherever the arm actually ended
    // up while this controller was inactive, causing a snap back toward the old position.
    this->declare_parameter("servo_controller_name", std::string{"rover_arm_velocity_controller"});

    home_button_index_ = this->get_parameter("home_button_index").as_int();
    controllers_to_toggle_ = this->get_parameter("controllers_to_toggle").as_string_array();
    servo_controller_name_ = this->get_parameter("servo_controller_name").as_string();
    
    if (controllers_to_toggle_.size() < 2) {
      RCLCPP_ERROR(this->get_logger(), "At least two controllers must be specified to toggle between them");
      return;
    }
    
    // Currently active controller index
    active_controller_index_ = 0;
    
    // Create subscriber for joy messages
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy2", 10, 
      std::bind(&ControllerSwitcher::joy_callback, this, std::placeholders::_1));
    
    // Create client for controller manager
    controller_switch_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");

    // Wait for the service to be available
    while (!controller_switch_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for controller_manager service...");
    }

    // Create client for moveit_servo's start_servo trigger
    start_servo_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");

    while (!start_servo_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for /servo_node/start_servo service...");
    }

    RCLCPP_INFO(this->get_logger(), "Controller switcher initialized.");
    RCLCPP_INFO(this->get_logger(), "Listening for home button press on button index: %d", home_button_index_);
    RCLCPP_INFO(this->get_logger(), "Currently active controller: %s", controllers_to_toggle_[active_controller_index_].c_str());
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Check if the home button is pressed (rising edge detection)
    if (msg->buttons.size() > static_cast<size_t>(home_button_index_)) {
      if (msg->buttons[home_button_index_] == 1 && !home_button_pressed_) {
        home_button_pressed_ = true;
        toggle_controllers();
      } else if (msg->buttons[home_button_index_] == 0) {
        home_button_pressed_ = false;
      }
    }
  }

  void toggle_controllers()
  {
    // Get current active and next controller
    const std::string current_controller = controllers_to_toggle_[active_controller_index_];
    active_controller_index_ = (active_controller_index_ + 1) % controllers_to_toggle_.size();
    const std::string next_controller = controllers_to_toggle_[active_controller_index_];

    if (next_controller == servo_controller_name_) {
      // moveit_servo keeps running (and publishing) the whole time regardless of which
      // ros2_control controller is active. start_servo re-syncs its internal state/filters
      // from the robot's actual current position before we hand it control again, so it
      // doesn't resume from wherever it last thought the arm was.
      RCLCPP_INFO(this->get_logger(), "Calling start_servo before switching to %s", next_controller.c_str());
      auto trigger_request = std::make_shared<std_srvs::srv::Trigger::Request>();
      start_servo_client_->async_send_request(
        trigger_request,
        [this, current_controller, next_controller](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {
          if (!future.get()->success) {
            RCLCPP_WARN(this->get_logger(), "start_servo call did not report success, switching controllers anyway");
          }
          send_switch_controller_request(current_controller, next_controller);
        }
      );
    } else {
      send_switch_controller_request(current_controller, next_controller);
    }
  }

  void send_switch_controller_request(const std::string& current_controller, const std::string& next_controller)
  {
    RCLCPP_INFO(this->get_logger(), "Switching controllers: %s -> %s",
                current_controller.c_str(), next_controller.c_str());

    // Create request
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->start_controllers = {next_controller};
    request->stop_controllers = {current_controller};
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    request->start_asap = false;

    // Send request
    controller_switch_client_->async_send_request(
      request,
      [this, current_controller, next_controller](
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future)
      {
        if (future.get()->ok) {
          RCLCPP_INFO(this->get_logger(), "Successfully switched controllers: %s -> %s",
                      current_controller.c_str(), next_controller.c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to switch controllers");
          // Revert active controller index since the switch failed
          active_controller_index_ = (active_controller_index_ + controllers_to_toggle_.size() - 1) % controllers_to_toggle_.size();
        }
      }
    );
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_switch_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;

  std::vector<std::string> controllers_to_toggle_;
  std::string servo_controller_name_;
  int home_button_index_;
  size_t active_controller_index_;
  bool home_button_pressed_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerSwitcher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
