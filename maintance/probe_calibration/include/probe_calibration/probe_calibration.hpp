#ifndef PROBE_CALIBRATION__PROBE_CALIBRATION_HPP_
#define PROBE_CALIBRATION__PROBE_CALIBRATION_HPP_

// ___CPP___
#include <mutex>
#include <thread>

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

// ___Avena___
#include <custom_interfaces/action/simple_action.hpp>
#include <helpers_vision/helpers_vision.hpp>
#include <helpers_commons/helpers_commons.hpp>

// ___Package___
#include "probe_calibration/visibility_control.h"

namespace probe_calibration
{
  using ProbeCalibrationAction = custom_interfaces::action::SimpleAction;
  using GoalHandleProbeCalibration = rclcpp_action::ServerGoalHandle<ProbeCalibrationAction>;

  class ProbeCalibration : public rclcpp::Node
  {
  public:
    explicit ProbeCalibration(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~ProbeCalibration() = default;

  private:
    void _calibrateWithProbe(const std::shared_ptr<GoalHandleProbeCalibration> goal_handle);
    std::optional<geometry_msgs::msg::TransformStamped> _lookupTransform(const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &timestamp = rclcpp::Time(0));
    size_t _getIntParameter();
    int _readingParamsCallback();
    void _displayTransform(const geometry_msgs::msg::TransformStamped &tf, std::ostream &out);
    void _displayTransform(const Eigen::Affine3d &tf, std::ostream &out);
    void _computeChosenPointPosition(Eigen::Affine3d &camera_to_marker_tf, cv::Point &point, Eigen::Vector3d &out_marker_moved_to_probe);
    void _computeCameraPositionFromThreePoints(Eigen::Vector3d robot_A, Eigen::Vector3d camera_A,Eigen::Vector3d robot_B, Eigen::Vector3d camera_B,Eigen::Vector3d robot_C, Eigen::Vector3d camera_C, Eigen::Affine3d &out_pose);

    rclcpp_action::Server<ProbeCalibrationAction>::SharedPtr _probe_calibration_action_server;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _camera_sub;

    std::unique_ptr<tf2_ros::TransformListener> _transform_listener;
    std::unique_ptr<tf2_ros::Buffer> _transforms_buffer;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> _static_transform_broadcaster;

    size_t _nr_samples;
    std::string _rgb_frame_name;
    std::string _probe_frame_name;
    std::string _base_frame_name;
    double _chessboard_square_size = -1;
    std::string _marker_frame_name;
  };

} // namespace probe_calibration

#endif // PROBE_CALIBRATION__PROBE_CALIBRATION_HPP_
