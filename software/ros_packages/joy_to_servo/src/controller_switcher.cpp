#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

using namespace std::chrono_literals;

class ControllerSwitcher : public rclcpp::Node
{
public:
  ControllerSwitcher()
  : Node("controller_switcher")
  {
    // Parameters
    this->declare_parameter("home_button_index", 8);
    this->declare_parameter("controllers_to_toggle", std::vector<std::string>{"rover_arm_controller", "rover_arm_controller_moveit"});
    
    home_button_index_ = this->get_parameter("home_button_index").as_int();
    controllers_to_toggle_ = this->get_parameter("controllers_to_toggle").as_string_array();
    
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
    const std::string& current_controller = controllers_to_toggle_[active_controller_index_];
    active_controller_index_ = (active_controller_index_ + 1) % controllers_to_toggle_.size();
    const std::string& next_controller = controllers_to_toggle_[active_controller_index_];
    
    RCLCPP_INFO(this->get_logger(), "Switching controllers: %s -> %s", 
                current_controller.c_str(), next_controller.c_str());
    
    // Create request
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->start_controllers = {next_controller};
    request->stop_controllers = {current_controller};
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    request->start_asap = false;
    
    // Send request
    auto future = controller_switch_client_->async_send_request(
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
  
  std::vector<std::string> controllers_to_toggle_;
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
