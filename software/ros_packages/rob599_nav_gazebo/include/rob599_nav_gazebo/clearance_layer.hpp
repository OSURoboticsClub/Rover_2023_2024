#pragma once

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace rob599_nav_gazebo
{

class ClearanceLayer : public nav2_costmap_2d::Layer
{
public:
  ClearanceLayer() = default;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  double robot_height_;
  sensor_msgs::msg::PointCloud2 latest_cloud_;
  std::mutex cloud_mutex_;
  bool received_cloud_ = false;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

} // namespace rob599_nav_gazebo