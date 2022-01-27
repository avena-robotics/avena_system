#ifndef SAVE_TRAJECTORY__SAVE_TRAJECTORY_HPP
#define SAVE_TRAJECTORY__SAVE_TRAJECTORY_HPP

// CPP
#include <memory>
#include <fstream>
#include <mutex>
#include <filesystem>

// ROS2
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

constexpr double JOINT_POSMAX = M_PI;
constexpr double JOINT_VELMAX = 2 * M_PI;
constexpr double JOINT_ACCMAX = 4 * M_PI;
constexpr double JOINT_TORQUEMAX = 256.0;
constexpr double MAX_INT16 = 32767.0;

namespace save_trajectory
{
  static const rclcpp::Logger LOGGER = rclcpp::get_logger("avena_arm_saving_trajectory");

  class SaveTrajectory : public rclcpp::Node
  {
  public:
    using Action = control_msgs::action::FollowJointTrajectory;
    using GoalHandleAction = rclcpp_action::ServerGoalHandle<Action>;

    explicit SaveTrajectory(rclcpp::NodeOptions options);
    ~SaveTrajectory() = default;

  private:
    rclcpp_action::Server<Action>::SharedPtr _action_server;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _dummy_joint_state_pub;
    rclcpp::TimerBase::SharedPtr _joint_states_timer;
    sensor_msgs::msg::JointState _current_joint_states;
    std::mutex _current_joint_states_mtx;
    std::filesystem::path _base_path;
    bool _run_joint_state_pub = true;

    rclcpp_action::GoalResponse _handleGoal(const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const Action::Goal> /*goal*/);
    rclcpp_action::CancelResponse _handleCancel(const std::shared_ptr<GoalHandleAction> /*goal_handle*/);
    void _handleAccepted(const std::shared_ptr<GoalHandleAction> goal_handle);
    void _execute(const std::shared_ptr<GoalHandleAction> goal_handle);
  };

} // namespace save_trajectory

#endif // SAVE_TRAJECTORY__SAVE_TRAJECTORY_HPP
