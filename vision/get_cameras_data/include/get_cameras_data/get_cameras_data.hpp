#ifndef GET_CAMERAS_DATA__GET_CAMERAS_DATA_HPP_
#define GET_CAMERAS_DATA__GET_CAMERAS_DATA_HPP_

// ___ROS2___
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_listener.h>

// ___Avena___
#include "custom_interfaces/msg/rgb_images.hpp"
#include "custom_interfaces/msg/depth_images.hpp"
#include "custom_interfaces/msg/ptclds.hpp"
#include "helpers_vision/helpers_vision.hpp"
#include "helpers_commons/helpers_commons.hpp"
// ___Package___
#include "get_cameras_data/visibility_control.h"

namespace get_cameras_data
{
  class GetCamerasData : public rclcpp::Node
  {
  public:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                                       sensor_msgs::msg::Image,
                                                                       sensor_msgs::msg::Image,
                                                                       sensor_msgs::msg::Image>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

    GET_CAMERAS_DATA_PUBLIC
    explicit GetCamerasData(const rclcpp::NodeOptions &options);

    GET_CAMERAS_DATA_PUBLIC
    virtual ~GetCamerasData();

  private:
    helpers::Watchdog::SharedPtr _watchdog;

    GET_CAMERAS_DATA_LOCAL
    void _synchronizedTopicsCallback(const sensor_msgs::msg::Image::ConstSharedPtr &cam1_rgb, const sensor_msgs::msg::Image::ConstSharedPtr &cam1_depth,
                                     const sensor_msgs::msg::Image::ConstSharedPtr &cam2_rgb, const sensor_msgs::msg::Image::ConstSharedPtr &cam2_depth);

    // Synchronization members
    message_filters::Subscriber<sensor_msgs::msg::Image> _rgb1_image_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> _depth1_image_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> _rgb2_image_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> _depth2_image_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _ptclds_sub;
    std::unique_ptr<Synchronizer> _syncApproximate;

    rclcpp::Publisher<custom_interfaces::msg::RgbImages>::SharedPtr _rgb_images_pub;
    rclcpp::Publisher<custom_interfaces::msg::DepthImages>::SharedPtr _depth_images_pub;
    rclcpp::Publisher<custom_interfaces::msg::Ptclds>::SharedPtr _ptclds_pub;

    const std::string _camera1_frame = "";
    const std::string _camera2_frame = "";

    const std::string _rgb_topic = "/rgb/image_raw";
    const std::string _depth_topic = "/depth_to_rgb/image_raw";
    const std::string _ptcld_topic = "/points2";
  };

} // namespace get_cameras_data

#endif // GET_CAMERAS_DATA__GET_CAMERAS_DATA_HPP_
