#ifndef OCTOMAP_FILTER_DEBUG__COMPONENT_HPP_
#define OCTOMAP_FILTER_DEBUG__COMPONENT_HPP_

#include "visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include <mysql_connector.h>
#include <assert.h>
#include <sstream>

// #include "custom_interfaces/msg/octomap_filter.hpp"
#include "custom_interfaces/msg/filtered_scene_octomap.hpp"

#include "helpers_vision/helpers_vision.hpp"

#include <memory>

namespace ros2mysql
{

  class OctomapFilter : public rclcpp::Node
  {
  public:
    OCTOMAP_FILTER_PUBLIC
    explicit OctomapFilter(const rclcpp::NodeOptions &options);

  private:
    /**;
   * Configures component.
   *
   * Declares parameters and configures video capture.
   */
    OCTOMAP_FILTER_PUBLIC
    void configure();

    /**;
   * Declares the parameter using rcl_interfaces.
   */
    OCTOMAP_FILTER_PUBLIC
    void initialize_parameters();

    std::unique_ptr<MysqlConnector> db_;
    std::string host_;
    std::string port_;
    std::string db_name_;
    std::string username_;
    std::string password_;
    bool debug_;
    rclcpp::Subscription<custom_interfaces::msg::FilteredSceneOctomap>::SharedPtr sub_;
  };

} // namespace ros2mysql

#endif // OCTOMAP_FILTER_DEBUG__COMPONENT_HPP_
