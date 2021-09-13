#ifndef MOTION_PLANNING__MOTION_PLANNING_HPP_
#define MOTION_PLANNING__MOTION_PLANNING_HPP_

// ___Package___
#include "motion_planning/commons.hpp"

namespace motion_planning
{
  class MotionPlanning : public rclcpp::Node, public helpers::WatchdogInterface
  {
  public:
    explicit MotionPlanning(const rclcpp::NodeOptions &options);
    virtual ~MotionPlanning();
    virtual void initNode() override;
    virtual void shutDownNode() override;

  private:
    // ___Methods___
    rclcpp_action::GoalResponse _handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MotionPlanningAction::Goal> goal);
    rclcpp_action::CancelResponse _handleCancel(const std::shared_ptr<GoalHandleMotionPlanningAction> goal_handle);
    void _handleAccepted(const std::shared_ptr<GoalHandleMotionPlanningAction> goal_handle);
    void _execute(const std::shared_ptr<GoalHandleMotionPlanningAction> goal_handle);

    void _initialize();
    void _shutdown();

    ReadData::SharedPtr _readData();
    void _writeData(const WriteData::SharedPtr write_data);

    // ___Attributes___
    helpers::Watchdog::SharedPtr _watchdog;

    generate_path::GeneratePath::UniquePtr _generate_path_handler;
    generate_trajectory::GenerateTrajectory::UniquePtr _generate_trajectory_handler;
    
    // Action servers and data store clients
    rclcpp_action::Server<MotionPlanningAction>::SharedPtr _action_server_motion_planning;
    rclcpp::Client<TrajectoryInsert>::SharedPtr _trajectory_insert_client;
    rclcpp::Client<OctomapSelect>::SharedPtr _octomap_select_client;
    rclcpp::Client<MovementSequenceSelect>::SharedPtr _movement_sequence_select_client;

    // Debug publishers
    rclcpp::Publisher<generate_path::GeneratedPath>::SharedPtr _generated_path_pub_debug;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _generated_trajectory_pub_debug;
  };

} // namespace motion_planning

#endif // MOTION_PLANNING__MOTION_PLANNING_HPP_
