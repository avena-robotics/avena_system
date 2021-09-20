#ifndef GET_CAMERAS_DATA__GET_CAMERAS_DATA_HPP_
#define GET_CAMERAS_DATA__GET_CAMERAS_DATA_HPP_

#include <tuple>
#include <cstdarg>
#include <iostream>
// ___ROS2___
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_listener.h>

// ___Avena___
#include "helpers_vision/helpers_vision.hpp"
#include "helpers_commons/helpers_commons.hpp"
// ___Package___
#include "get_cameras_data/visibility_control.h"

#include "get_cameras_data/multi_sub_image.hpp"
#include "get_cameras_data/multi_sub_ptcld.hpp"



namespace get_cameras_data
{
  class GetCamerasData : public rclcpp::Node, public helpers::WatchdogInterface
  {
  public:

    GET_CAMERAS_DATA_PUBLIC
    explicit GetCamerasData(const rclcpp::NodeOptions &options);

    GET_CAMERAS_DATA_PUBLIC
    virtual ~GetCamerasData();

    virtual void initNode() override;
    virtual void shutDownNode() override;

  private:
    helpers::Watchdog::SharedPtr _watchdog;

    GET_CAMERAS_DATA_LOCAL




    void _getParametersFromServer();

    synchronizers_image::image_subscriptions _rgb_image_subs;
    synchronizers_image::image_subscriptions _depth_image_subs;
    synchronizers_ptcld::ptclds_subscriptions _ptcld_subs;

    std::shared_ptr<synchronizers_image::Images> _sync_rgb;
    std::shared_ptr<synchronizers_image::Images> _sync_depth;
    std::shared_ptr<synchronizers_ptcld::Ptclds> _sync_ptcld;


    size_t _cameras_amount;

    const std::string _camera_frame_prefix = "/camera_";
    const std::string _rgb_topic = "/rgb/image_raw";
    const std::string _depth_topic = "/depth_to_rgb/image_raw";
    const std::string _ptcld_topic = "/points2";
  };

} // namespace get_cameras_data

#endif // GET_CAMERAS_DATA__GET_CAMERAS_DATA_HPP_
