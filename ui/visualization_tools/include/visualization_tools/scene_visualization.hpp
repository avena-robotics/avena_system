#ifndef VISUALIZATION_TOOLS__SCENE_VISUALIZATION_HPP_
#define VISUALIZATION_TOOLS__SCENE_VISUALIZATION_HPP_

// ___CPP___
#include <filesystem>

// ___Other___
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <urdf_parser/urdf_parser.h>

// ___Avena___
#include <helpers_commons/helpers_commons.hpp>
#include <helpers_vision/helpers_vision.hpp>
#include <custom_interfaces/srv/data_store_scene_select.hpp>

// ___Package___
#include "visualization_tools/visibility_control.h"

namespace visualization_tools
{
  using OctomapChangeFlag = std_msgs::msg::String;
  using OctomapSelect = custom_interfaces::srv::DataStoreSceneSelect;

  class SceneVisualization : public rclcpp::Node
  {
  public:
    explicit SceneVisualization(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    int _getParametersFromServer();
    void _octomapUpdated(const OctomapChangeFlag::SharedPtr octomap_change_flag_msg);
    int _createMeshFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, const float &grid_size, std::vector<pcl::Vertices> &out_triangles);

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _table_marker_pub;
    rclcpp::Subscription<OctomapChangeFlag>::SharedPtr _octomap_changed_sub;
    // rclcpp::TimerBase::SharedPtr _static_markers_timer;
    rclcpp::Client<OctomapSelect>::SharedPtr _octomap_select_client;
    nlohmann::json _areas_parameters;
  };

} // namespace visualization_tools

#endif // VISUALIZATION_TOOLS__SCENE_VISUALIZATION_HPP_
