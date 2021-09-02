#ifndef MOTION_PLANNING__COMMONS_HPP_
#define MOTION_PLANNING__COMMONS_HPP_

// ___CPP___
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// ___Avena___
#include <helpers_vision/helpers_vision.hpp>
#include <helpers_commons/helpers_commons.hpp>
#include <generate_path/generate_path.hpp>
#include <generate_trajectory/generate_trajectory.hpp>

// ___Avena - data store services___
#include <custom_interfaces/srv/data_store_trajectory_insert.hpp>
#include <custom_interfaces/srv/data_store_scene_select.hpp>
#include <custom_interfaces/srv/data_store_movement_sequence_select.hpp>
#include <custom_interfaces/action/simple_action.hpp>
#include <custom_interfaces/msg/generated_path.hpp>

// ___Package___
#include "motion_planning/visibility_control.h"

namespace motion_planning
{
  using namespace std::chrono_literals;

  using TrajectoryInsert = custom_interfaces::srv::DataStoreTrajectoryInsert;
  using OctomapSelect = custom_interfaces::srv::DataStoreSceneSelect;
  using MovementSequenceSelect = custom_interfaces::srv::DataStoreMovementSequenceSelect;
  using EndEffectorPose = custom_interfaces::msg::EndEffectorPose;

  using MotionPlanningAction = custom_interfaces::action::SimpleAction;
  using GoalHandleMotionPlanningAction = rclcpp_action::ServerGoalHandle<MotionPlanningAction>;
  

  struct ReadData
  {
    std::vector<EndEffectorPose> movement_sequence;
    pcl::PointCloud<pcl::PointXYZ>::Ptr octomap;

    using UniquePtr = std::unique_ptr<ReadData>;
    using SharedPtr = std::shared_ptr<ReadData>;
  };

  struct WriteData
  {
    trajectory_msgs::msg::JointTrajectory::SharedPtr trajectory;

    using UniquePtr = std::unique_ptr<WriteData>;
    using SharedPtr = std::shared_ptr<WriteData>;
  };

} // namespace motion_planning

#endif // MOTION_PLANNING__COMMONS_HPP_
