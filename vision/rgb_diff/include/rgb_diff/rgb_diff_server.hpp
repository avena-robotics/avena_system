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
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
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
#include "helpers_commons/structures.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/bgsegm.hpp>

//PCL
#include <pcl/io/ply_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <robot_mask/robot_mask.hpp>

#define WORLD "world"


namespace rgb_diff_action_server
{
  struct AvenaMesh
    {
        AvenaMesh()
            : cloud(new pcl::PointCloud<pcl::PointXYZ>)
        {
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        pcl::PolygonMesh mesh;
    };

  struct CameraParameters
  {
      float cx;
      float cy;
      float fx;
      float fy;

      size_t width;
      size_t height;
  };


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
    int _rgbdiff_scene_change_threshold = 50;
    std::shared_ptr<cv::Mat> background_substractor(cv::Mat &former, cv::Mat &latter, cv::Mat& target);
    void set_security_area_masks();


    std::vector<std::string> _removeRobotPrefix(std::vector<std::string> link_names, std::string robot_prefix);


    //ROS
    rclcpp_action::Server<RgbDiffAction>::SharedPtr _action_server;
    rclcpp_action::GoalResponse _handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const RgbDiffAction::Goal> goal);
    rclcpp_action::CancelResponse _handle_cancel(const std::shared_ptr<GoalHandleRgbDiffAction> goal_handle);
    void _handle_accepted(const std::shared_ptr<GoalHandleRgbDiffAction> goal_handle);
    void _execute(const std::shared_ptr<GoalHandleRgbDiffAction> goal_handle);
    std::unique_ptr<tf2_ros::TransformListener> _transform_listener;
    std::unique_ptr<tf2_ros::Buffer> _transforms_buffer;

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
    cv::Mat bckgrnd_robot_mask_cam1, bckgrnd_robot_mask_cam2;

    sensor_msgs::msg::Image ros_mask_cam1, ros_mask_cam2;
    std::map<std::string, AvenaMesh> _meshes;
    std::vector<std::string> _robot_links_names;
    helpers::commons::RobotInfo _robot_info;
    // custom_interfaces::msg::Cameras::SharedPtr _rgbdiff_threshold;

    builtin_interfaces::msg::Time _last_processed_msg_timestamp;


robot_mask::RobotMask *_robot_mask_object;


  };

} // namespace rgb_diff_action_server

#endif
