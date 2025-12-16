#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <Eigen/Dense>

class ManipulabilityNode : public rclcpp::Node
{
public:
  ManipulabilityNode()
  : Node("manipulability_node")
  {
    // robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(this->get_node_base_interface(), this->get_node_parameters_interface(), "robot_description");
    // robot_model_ = robot_model_loader_->getModel();

    // if (!robot_model_)
    // {
    //   RCLCPP_ERROR(get_logger(), "Failed to load robot model");
    //   return;
    // }

    // robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    // joint_model_group_ = robot_model_->getJointModelGroup("your_arm_group_name");

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&ManipulabilityNode::computeManipulability, this));
  }

private:
  void computeManipulability()
  {
    // Update robot state from MoveIt2 (you should subscribe to joint state or use planning scene monitor)
    // For now, we assume robot_state_ is valid and updated

    const Eigen::MatrixXd& jacobian = robot_state_->getJacobian(joint_model_group_);
    
    // Yoshikawa manipulability measure
    double manipulability = std::sqrt((jacobian * jacobian.transpose()).determinant());

    RCLCPP_INFO(get_logger(), "Manipulability: %f", manipulability);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  const moveit::core::JointModelGroup* joint_model_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManipulabilityNode>());
  rclcpp::shutdown();
  return 0;
}
