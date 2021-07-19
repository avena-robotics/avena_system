#ifndef GENERATE_PATH__GENERATE_PATH_HPP_
#define GENERATE_PATH__GENERATE_PATH_HPP_

// ___Bullet___
// #include "/home/avena/repos/bullet3_install/include/bullet/RobotSimulator/b3RobotSimulatorClientAPI.h"
// #include "/home/avena/repos/bullet3-3.17/examples/RobotSimulator/b3RobotSimulatorClientAPI.h"

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// ___Avena___
#include <custom_interfaces/action/generate_path_pose.hpp>
#include <custom_interfaces/msg/generated_path.hpp>
#include <helpers_commons/helpers_commons.hpp>

// ___Package___
#include "generate_path/visibility_control.h"

enum class ReturnCode
{
  SUCCESS = 0,
  FAILURE
};

namespace generate_path
{
  class GeneratePath : public rclcpp::Node, public helpers::WatchdogInterface
  {
  public:
    using GeneratePathPose = custom_interfaces::action::GeneratePathPose;
    using GoalHandleGeneratePathPose = rclcpp_action::ServerGoalHandle<GeneratePathPose>;
    using ArmConfiguration = std::vector<float>;

    explicit GeneratePath(const rclcpp::NodeOptions &options);
    virtual ~GeneratePath();
    virtual void initNode() override;
    virtual void shutDownNode() override;

  private:
    // ___Methods___
    // ___Go to end effector pose___
    rclcpp_action::GoalResponse _handleGoalPose(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GeneratePathPose::Goal> goal);
    rclcpp_action::CancelResponse _handleCancelPose(const std::shared_ptr<GoalHandleGeneratePathPose> goal_handle);
    void _handleAcceptedPose(const std::shared_ptr<GoalHandleGeneratePathPose> goal_handle);
    void _executePose(const std::shared_ptr<GoalHandleGeneratePathPose> goal_handle);
    ReturnCode _initialize();
    ReturnCode _shutdown();

    // ___Attributes___
    helpers::Watchdog::SharedPtr _watchdog;
    rclcpp_action::Server<GeneratePathPose>::SharedPtr _action_server_pose;
    rclcpp::Publisher<custom_interfaces::msg::GeneratedPath>::SharedPtr _generated_path_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_state_sub;
    sensor_msgs::msg::JointState::SharedPtr _current_joint_states;
    // std::shared_ptr<b3RobotSimulatorClientAPI> _bullet_client;

  };

} // namespace generate_path

#endif // GENERATE_PATH__GENERATE_PATH_HPP_
