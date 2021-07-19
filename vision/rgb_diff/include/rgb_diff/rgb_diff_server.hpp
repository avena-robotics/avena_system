#ifndef RGB_DIFF_ACTION_SERVER_COMPONENT_HPP
#define RGB_DIFF_ACTION_SERVER_COMPONENT_HPP

// ___CPP___
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/parameter.hpp>

// __ROS__
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "rclcpp_components/register_node_macro.hpp"

// ___AVENA___
// #include "detect_action_server/visibility_control.h"
#include "custom_interfaces/action/simple_action.hpp"
#include "custom_interfaces/msg/rgb_images.hpp"
#include "custom_interfaces/msg/rgb_diff_result.hpp"
#include "custom_interfaces/msg/cameras_data.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <boost/beast/core.hpp>
#include <utility>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <cstdlib>
#include <iostream>
#include <string>
#include <opencv2/core.hpp>
#include <boost/asio/strand.hpp>
#include <memory>
#include <boost/program_options.hpp>
#include <nlohmann/json.hpp>
#include <helpers_vision/helpers_vision.hpp>
#include <helpers_commons/helpers_commons.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/bgsegm.hpp>

namespace rgb_diff_action_server
{
  using json = nlohmann::json;
  class RgbDiffActionServer : public rclcpp::Node, public helpers::WatchdogInterface
  {
  public:
    using RgbDiffAction = custom_interfaces::action::SimpleAction;
    using GoalHandleRgbDiffAction = rclcpp_action::ServerGoalHandle<RgbDiffAction>;

    explicit RgbDiffActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    void initNode();
    void shutDownNode();

  private:
    helpers::Watchdog::SharedPtr _watchdog;
    std::string _getParam(std::string param_name);
    std::string ip;
    std::string model;
    std::string detectron_error_body_key{"error_body"};
    int _rgbdiff_pixel_threshold =  70;
    int _rgbdiff_scene_change_threshold = 500;
    std::shared_ptr<cv::Mat> background_substractor(cv::Mat &former, cv::Mat &latter, cv::Mat& target);
    void set_security_area_masks();

    //ROS
    rclcpp_action::Server<RgbDiffAction>::SharedPtr _action_server;
    rclcpp_action::GoalResponse _handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const RgbDiffAction::Goal> goal);
    rclcpp_action::CancelResponse _handle_cancel(const std::shared_ptr<GoalHandleRgbDiffAction> goal_handle);
    void _handle_accepted(const std::shared_ptr<GoalHandleRgbDiffAction> goal_handle);
    void _execute(const std::shared_ptr<GoalHandleRgbDiffAction> goal_handle);

    // PUBLISHER
    rclcpp::Publisher<custom_interfaces::msg::RgbDiffResult>::SharedPtr _publisher_rgbdiff_result;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _publisher_security_area_changed;
    
    //SUBSCRIBERS
    rclcpp::Subscription<custom_interfaces::msg::CamerasData>::SharedPtr _rgb_images_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _rgbdiff_set_background_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _rgbdiff_pixel_threshold_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _rgbdiff_scene_change_threshold_sub;

    custom_interfaces::msg::CamerasData::SharedPtr _buffered_rgb_images = nullptr;
    cv::Mat _rgbdiff_background_cam1;
    cv::Mat _rgbdiff_background_cam2;
    cv::Mat cam1_rgb, cam2_rgb, result_cam1, result_cam2;
    cv::Mat result_cam1_rgb, result_cam2_rgb;
    cv::Mat _sec_area_cam1_mask, _sec_area_cam2_mask;
    sensor_msgs::msg::Image ros_mask_cam1, ros_mask_cam2;

    // custom_interfaces::msg::Cameras::SharedPtr _rgbdiff_threshold;

    builtin_interfaces::msg::Time _last_processed_msg_timestamp;
  };

} // namespace rgb_diff_action_server

#endif
