#ifndef HELPERS_VISION__VISUALIZATION_HPP_
#define HELPERS_VISION__VISUALIZATION_HPP_

// ___CPP___
#include <bitset>

// ___PCL___
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

namespace helpers
{
  namespace visualization
  {
    pcl::visualization::PCLVisualizer::Ptr visualize(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> point_clouds, std::vector<Eigen::Affine3f> affines = {}, pcl::visualization::PCLVisualizer::Ptr viewer = nullptr, std::string window_name = "");
  } // namespace visualization
} // namespace helpers

#endif // HELPERS_VISION__VISUALIZATION_HPP_
