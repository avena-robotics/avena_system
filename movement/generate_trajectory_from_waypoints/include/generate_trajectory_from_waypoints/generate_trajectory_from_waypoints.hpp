#ifndef GENERATE_TRAJECTORY_FROM_WAYPOINTS__GENERATE_TRAJECTORY_FROM_WAYPOINTS_HPP
#define GENERATE_TRAJECTORY_FROM_WAYPOINTS__GENERATE_TRAJECTORY_FROM_WAYPOINTS_HPP

// CPP
#include <fstream>
#include <filesystem>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <warehouse_ros/database_loader.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_srvs/srv/trigger.hpp>
#include <custom_interfaces/action/trigger.hpp>

namespace generate_trajectory_from_waypoints
{
  static const rclcpp::Logger LOGGER = rclcpp::get_logger("generate_trajectory_from_waypoints");
  static const std::string PLANNING_GROUP = "avena_arm";
  static constexpr int DB_CONNECT_TIMEOUT = 20; // seconds

  class GenerateTrajectoryFromWaypoints : public rclcpp::Node
  {
  public:
    using Action = custom_interfaces::action::Trigger;
    using GoalHandleAction = rclcpp_action::ServerGoalHandle<Action>;

    explicit GenerateTrajectoryFromWaypoints(rclcpp::NodeOptions options);
    virtual ~GenerateTrajectoryFromWaypoints() = default;

  private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _trajectory_pub;
    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _generate_traj_trig_srv;

    rclcpp_action::Server<Action>::SharedPtr _action_server;

    std::atomic<bool> _db_connected = false;
    std::unique_ptr<moveit_warehouse::RobotStateStorage> _robot_state_storage;
    
    rclcpp_action::GoalResponse _handleGoal(const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const Action::Goal> /*goal*/);
    rclcpp_action::CancelResponse _handleCancel(const std::shared_ptr<GoalHandleAction> /*goal_handle*/);
    void _handleAccepted(const std::shared_ptr<GoalHandleAction> goal_handle);
    void _generateTrajTrigExecute(const std::shared_ptr<GoalHandleAction> goal_handle);
  };

} // namespace generate_trajectory_from_waypoints

#endif // GENERATE_TRAJECTORY_FROM_WAYPOINTS__GENERATE_TRAJECTORY_FROM_WAYPOINTS_HPP
