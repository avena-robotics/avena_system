#ifndef GENERATE_TRAJECTORY__GENERATE_TRAJECTORY_HPP_
#define GENERATE_TRAJECTORY__GENERATE_TRAJECTORY_HPP_

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// ___Avena___
#include <custom_interfaces/action/simple_action.hpp>
#include <custom_interfaces/msg/generated_path.hpp>
#include <helpers_commons/helpers_commons.hpp>

// ___Package___
#include "path.h"
#include "trajectory.h"
#include "visibility_control.h"

namespace generate_trajectory
{
  enum class ReturnCode
  {
    SUCCESS = 0,
    FAILURE
  };

  using GenerateTrajectoryAction = custom_interfaces::action::SimpleAction;
  using GoalHandleGenerateTrajectory = rclcpp_action::ServerGoalHandle<GenerateTrajectoryAction>;
  using GeneratedPath = custom_interfaces::msg::GeneratedPath;

  class GenerateTrajectory : public rclcpp::Node, public helpers::WatchdogInterface
  {
  public:
    explicit GenerateTrajectory(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~GenerateTrajectory();
    virtual void initNode() override;
    virtual void shutDownNode() override;

  private:
    // ___Methods___
    int _generateTrajectoryFromPathSegment(trajectory_msgs::msg::JointTrajectory &segment_trajectory, trajectory_msgs::msg::JointTrajectory &out_trajectory);
    std::unique_ptr<trajectory_msgs::msg::JointTrajectory> _generateTrajectoryFromPath(const GeneratedPath::SharedPtr &generated_path);
    void _generateAcceleration(Eigen::VectorXd maxAcceleration, Trajectory &trajectory);
    rclcpp_action::GoalResponse _handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GenerateTrajectoryAction::Goal> goal);
    rclcpp_action::CancelResponse _handleCancel(const std::shared_ptr<GoalHandleGenerateTrajectory> goal_handle);
    void _handleAccepted(const std::shared_ptr<GoalHandleGenerateTrajectory> goal_handle);
    void _execute(const std::shared_ptr<GoalHandleGenerateTrajectory> goal_handle);
    void _convertToMsg(Trajectory &trajectory, const std::vector<std::string> &joint_names, trajectory_msgs::msg::JointTrajectory &out_trajectory);
    ReturnCode _getParametersFromServer();
    ReturnCode _initialize();
    ReturnCode _shutdown();

    // ___Attributes___
    helpers::Watchdog::SharedPtr _watchdog;
    rclcpp_action::Server<GenerateTrajectoryAction>::SharedPtr _action_server_generate;
    rclcpp::Subscription<GeneratedPath>::SharedPtr _generated_path_sub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _trajectory_pub;
    GeneratedPath::SharedPtr _current_generated_path;
    std::mutex _current_generated_path_mtx;
    helpers::commons::RobotInfo _robot_info;
    std::vector<std::string> _joint_names;
    double _time_step;
  };

} // namespace generate_trajectory

#endif
