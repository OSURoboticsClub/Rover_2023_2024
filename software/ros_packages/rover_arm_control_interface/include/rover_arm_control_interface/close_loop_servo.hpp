
#ifndef CLOSE_LOOP_SERVO_HPP
#define CLOSE_LOOP_SERVO_HPP
 
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


const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string EEF_FRAME_ID = "rover_arm_tool0";
const std::string BASE_FRAME_ID = "rover_arm_base_link";

namespace moveit_servo
{
class CloseLoopServo : public rclcpp::Node
{
  public:
    CloseLoopServo(const rclcpp::NodeOptions& options)

  private:
    void timer_callback(){}
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
} // namespace moveit servo

#endif //CLOSE_LOOP_SERVO_HPP