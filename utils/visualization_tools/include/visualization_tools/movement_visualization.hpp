#ifndef VISUALIZATION_TOOLS__MOVEMENT_VISUALIZATION_HPP_
#define VISUALIZATION_TOOLS__MOVEMENT_VISUALIZATION_HPP_

// ___CPP___
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// ___Avena___
#include <helpers_commons/helpers_commons.hpp>
#include <helpers_vision/helpers_vision.hpp>

// ___Package___
#include "visualization_tools/joint_state_to_transform.hpp"
#include "visualization_tools/visibility_control.h"

namespace visualization_tools
{
  class MovementVisualization : public rclcpp::Node
  {
  public:
    explicit MovementVisualization(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    void _callbackGeneratedPath(const trajectory_msgs::msg::JointTrajectory::SharedPtr trajectory);
    void _callbackTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr trajectory);
    geometry_msgs::msg::TransformStamped _getEndEffectorTransform(const std::vector<geometry_msgs::msg::TransformStamped> &fixed_transforms, const std::vector<geometry_msgs::msg::TransformStamped> &moving_transforms);
    int _getParametersFromServer();
    std::optional<geometry_msgs::msg::TransformStamped> _getWorldToBaseLinkTf();

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr _generated_path_sub;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr _trajectory_sub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _nav_path_pub;

    std::string _end_effector_name;
    helpers::commons::RobotInfo _robot_info;
    std::string _world_name = "world";
    geometry_msgs::msg::TransformStamped _world_to_base_link_tf;

    JointStateToTransform _joint_state_to_tf;
    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _static_tf_broadcaster;
  };

} // namespace visualization_tools

#endif // VISUALIZATION_TOOLS__MOVEMENT_VISUALIZATION_HPP_
