#ifndef ROBOT_MASK_ACTION_SERVER_COMPONENT_HPP
#define ROBOT_MASK_ACTION_SERVER_COMPONENT_HPP

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

#define WORLD "world"


namespace robot_mask
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
      std::string camera_frame;
      int width;
      int height;
  };


  using json = nlohmann::json;
  using camera_info = std::pair<Eigen::Affine3f, robot_mask::CameraParameters>;
  class RobotMask 
  {
  public:


     RobotMask(rclcpp::Node *node);
    void prepareRobotMask(builtin_interfaces::msg::Time timestamp, cv::Mat &out_mask_cam1, cv::Mat &out_mask_cam2);
    void waitForMasks();


  private:
    // //ROS
    std::unique_ptr<tf2_ros::TransformListener> _transform_listener;
    std::unique_ptr<tf2_ros::Buffer> _transforms_buffer;
    rclcpp::Node *_node;

    std::map<std::string, AvenaMesh> _meshes;
    std::vector<std::string> _robot_links_names;
    helpers::commons::RobotInfo _robot_info;
    std::vector<std::thread> _workers;
    bool _cam_params_loaded = false;
    camera_info _cam_1;
    camera_info _cam_2;
    bool _workers_running;




    std::vector<std::string> _removeRobotPrefix(std::vector<std::string> link_names, std::string robot_prefix);
    int _loadAvenaMeshes();
    void _getRobotMasks(const rclcpp::Time timestamp, std::string &link_name, cv::Mat &out_mask_cam1, cv::Mat &out_mask_cam2);
    int _lookupTransform(const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &timestamp, Eigen::Affine3f &out_transform);
    int _getCamerasParameters();
    void _joinThreads();



    // builtin_interfaces::msg::Time _last_processed_msg_timestamp;


  };

} // namespace robot_mask

#endif
