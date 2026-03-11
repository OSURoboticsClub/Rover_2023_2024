#include "rob599_nav_gazebo/elevation_layer.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <nav2_costmap_2d/cost_values.hpp>

using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace rob599_nav_gazebo
{

ElevationLayer::ElevationLayer()
: max_slope_(0.5),
  height_threshold_(0.25),
  cloud_received_(false)
{
}

ElevationLayer::~ElevationLayer()
{
}

void ElevationLayer::onInitialize()
{
  auto node = node_.lock();

  cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>());

  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points",
      rclcpp::SensorDataQoS(),
      std::bind(&ElevationLayer::pointCloudCallback, this, std::placeholders::_1));

  matchSize();
}

void ElevationLayer::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::fromROSMsg(*msg, *cloud_);
  cloud_received_ = true;
}

void ElevationLayer::processPointCloud()
{
  if (!cloud_received_) return;

  pcl::PointCloud<pcl::PointXYZ>::Ptr ground(
      new pcl::PointCloud<pcl::PointXYZ>());

  detectGroundRANSAC(cloud_, ground);

  if (ground->empty())
  {
    detectGroundNormals(cloud_, ground);
  }

  if (ground->empty())
  {
    detectGroundHeight(cloud_, ground);
  }

  computeElevationGrid(ground);
  computeSlope();
}

void ElevationLayer::detectGroundRANSAC(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground)
{
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.15);
  seg.setInputCloud(cloud);

  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty())
    return;

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*ground);
}

void ElevationLayer::detectGroundNormals(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());

  pcl::PointCloud<pcl::Normal>::Ptr normals(
      new pcl::PointCloud<pcl::Normal>());

  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setKSearch(20);
  ne.compute(*normals);

  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    if (fabs(normals->points[i].normal_z) > 0.9)
    {
      ground->points.push_back(cloud->points[i]);
    }
  }
}

void ElevationLayer::detectGroundHeight(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground)
{
  float min_z = std::numeric_limits<float>::max();

  for (auto & p : cloud->points)
  {
    if (p.z < min_z)
      min_z = p.z;
  }

  for (auto & p : cloud->points)
  {
    if (fabs(p.z - min_z) < height_threshold_)
    {
      ground->points.push_back(p);
    }
  }
}

void ElevationLayer::computeElevationGrid(
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground)
{
  auto * costmap = layered_costmap_->getCostmap();

  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();

  elevation_grid_.assign(size_x, std::vector<float>(size_y, NAN));

  for (auto & p : ground->points)
  {
    unsigned int mx, my;

    if (costmap->worldToMap(p.x, p.y, mx, my))
    {
      elevation_grid_[mx][my] = p.z;
    }
  }
}

void ElevationLayer::computeSlope()
{
  auto * costmap = layered_costmap_->getCostmap();

  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();

  slope_grid_.assign(size_x, std::vector<float>(size_y, 0.0));

  for (unsigned int i = 1; i < size_x - 1; i++)
  {
    for (unsigned int j = 1; j < size_y - 1; j++)
    {
      float dzdx =
        elevation_grid_[i+1][j] - elevation_grid_[i-1][j];

      float dzdy =
        elevation_grid_[i][j+1] - elevation_grid_[i][j-1];

      slope_grid_[i][j] = sqrt(dzdx*dzdx + dzdy*dzdy);
    }
  }
}

void ElevationLayer::updateBounds(
    double,
    double,
    double,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y)
{
  *min_x = std::numeric_limits<double>::lowest();
  *min_y = std::numeric_limits<double>::lowest();
  *max_x = std::numeric_limits<double>::max();
  *max_y = std::numeric_limits<double>::max();
}

void ElevationLayer::updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j)
{
  processPointCloud();

  for (int i = min_i; i < max_i; i++)
  {
    for (int j = min_j; j < max_j; j++)
    {
      if (slope_grid_[i][j] > max_slope_)
      {
        master_grid.setCost(i, j, LETHAL_OBSTACLE);
      }
    }
  }
}

void ElevationLayer::reset()
{
  cloud_received_ = false;
}

bool ElevationLayer::isClearable()
{
  return false;
}

}  // namespace rob599_nav_gazebo

PLUGINLIB_EXPORT_CLASS(
  rob599_nav_gazebo::ElevationLayer,
  nav2_costmap_2d::Layer)