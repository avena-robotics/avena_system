#ifndef VISUALIZATION_TOOLS__SCENE_VISUALIZATION_HPP_
#define VISUALIZATION_TOOLS__SCENE_VISUALIZATION_HPP_

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

// ___Avena___
#include <helpers_commons/helpers_commons.hpp>
#include <helpers_vision/helpers_vision.hpp>

// ___Package___
#include "visualization_tools/visibility_control.h"

namespace visualization_tools
{
  class SceneVisualization : public rclcpp::Node
  {
  public:
    explicit SceneVisualization(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    int _getParametersFromServer();
    int _publishStaticMarkers();
    int _initializeMarkersPublishers();

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _table_marker_pub;
    rclcpp::TimerBase::SharedPtr _static_markers_timer;
    nlohmann::json _areas_parameters;
  };

} // namespace visualization_tools

#endif // VISUALIZATION_TOOLS__SCENE_VISUALIZATION_HPP_
