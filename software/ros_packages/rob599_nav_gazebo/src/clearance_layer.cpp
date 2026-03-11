#include "rob599_nav_gazebo/clearance_layer.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

using rcl_interfaces::msg::ParameterType;

PLUGINLIB_EXPORT_CLASS(rob599_nav_gazebo::ClearanceLayer, nav2_costmap_2d::Layer)

namespace rob599_nav_gazebo
{

void ClearanceLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  RCLCPP_INFO(rclcpp::get_logger("ClearanceLayer"), "Initializing ClearanceLayer");
  declareParameter("robot_height", rclcpp::ParameterValue(0.5));
  node->get_parameter(name_ + "." + "robot_height", robot_height_);

  // Subscribe to the same pointcloud topic that VoxelLayer would use
  std::string topic = "pointcloud";
  node->declare_parameter("pointcloud_topic", topic);
  node->get_parameter("pointcloud_topic", topic);

  auto qos = rclcpp::SensorDataQoS();
  sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, qos,
      std::bind(&ClearanceLayer::pointcloudCallback, this, std::placeholders::_1));
}

void ClearanceLayer::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  latest_cloud_ = *msg;
  received_cloud_ = true;
}

void ClearanceLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                  double* min_x, double* min_y, double* max_x, double* max_y)
{
  // Cover entire local costmap
  *min_x = 0;
  *min_y = 0;
  *max_x = layered_costmap_->getCostmap()->getSizeInCellsX() *
           layered_costmap_->getCostmap()->getResolution();
  *max_y = layered_costmap_->getCostmap()->getSizeInCellsY() *
           layered_costmap_->getCostmap()->getResolution();
}

void ClearanceLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                                 int min_i, int min_j, int max_i, int max_j)
{
  if (!received_cloud_) {
    // No data yet
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud_copy;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud_copy = latest_cloud_;
  }

  double resolution = master_grid.getResolution();
  double origin_x = master_grid.getOriginX();
  double origin_y = master_grid.getOriginY();
  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();

  // Iterate through each point in the cloud
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_copy, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_copy, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_copy, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    double px = *iter_x;
    double py = *iter_y;
    double pz = *iter_z;

    if (pz > robot_height_) {
      continue; // ignore high points
    }

    int mx = (int)((px - origin_x) / resolution);
    int my = (int)((py - origin_y) / resolution);

    if (mx >= 0 && mx < (int)size_x && my >= 0 && my < (int)size_y) {
      master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

} // namespace rob599_nav_gazebo