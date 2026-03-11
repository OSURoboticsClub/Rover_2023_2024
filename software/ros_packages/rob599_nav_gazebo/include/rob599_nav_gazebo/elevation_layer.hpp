#ifndef ROB599_NAV_GAZEBO__ELEVATION_LAYER_HPP_
#define ROB599_NAV_GAZEBO__ELEVATION_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rclcpp/rclcpp.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rob599_nav_gazebo
{

class ElevationLayer : public nav2_costmap_2d::Layer
{
public:
  ElevationLayer();
  virtual ~ElevationLayer();

  virtual void onInitialize();
  virtual void updateBounds(
      double robot_x,
      double robot_y,
      double robot_yaw,
      double * min_x,
      double * min_y,
      double * max_x,
      double * max_y);

  virtual void updateCosts(
      nav2_costmap_2d::Costmap2D & master_grid,
      int min_i,
      int min_j,
      int max_i,
      int max_j);

  virtual void reset();

private:

  void pointCloudCallback(
      const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void processPointCloud();

  void detectGroundRANSAC(
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
      pcl::PointCloud<pcl::PointXYZ>::Ptr ground);

  void detectGroundNormals(
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
      pcl::PointCloud<pcl::PointXYZ>::Ptr ground);

  void detectGroundHeight(
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
      pcl::PointCloud<pcl::PointXYZ>::Ptr ground);

  void computeElevationGrid(
      pcl::PointCloud<pcl::PointXYZ>::Ptr ground);

  void computeSlope();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

  std::vector<std::vector<float>> elevation_grid_;
  std::vector<std::vector<float>> slope_grid_;

  double max_slope_;
  double height_threshold_;

  bool cloud_received_;
};

}  // namespace rob599_nav_gazebo

#endif