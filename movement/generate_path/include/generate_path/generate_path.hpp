#ifndef GENERATE_PATH__GENERATE_PATH_HPP_
#define GENERATE_PATH__GENERATE_PATH_HPP_

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <opencv2/opencv.hpp>

// ___Avena___
#include <custom_interfaces/action/generate_path_pose.hpp>
#include <custom_interfaces/msg/generated_path.hpp>
#include <helpers_commons/helpers_commons.hpp>
#include <helpers_vision/helpers_vision.hpp>
#include <bullet_client/b3RobotSimulatorClientAPI.h>

// ___Package___
#include "generate_path/visibility_control.h"
#include "generate_path/commons.hpp"
#include "generate_path/planner.hpp"

namespace generate_path
{
  using GeneratePathPose = custom_interfaces::action::GeneratePathPose;
  using GoalHandleGeneratePathPose = rclcpp_action::ServerGoalHandle<GeneratePathPose>;

  class GeneratePath : public rclcpp::Node, public helpers::WatchdogInterface
  {
  public:
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
    ArmConfiguration _getJointStatesFromTopic(const sensor_msgs::msg::JointState::SharedPtr &joint_states);
    ReturnCode _getParametersFromServer();
    void _convertPathSegmentToTrajectoryMsg(const std::vector<ArmConfiguration> &path, trajectory_msgs::msg::JointTrajectory &path_segment);
    ArmConfiguration _calculateGoalStateFromEndEffectorPose(const geometry_msgs::msg::Pose &end_effector_pose, const sensor_msgs::msg::JointState::SharedPtr &current_joint_states);
    ReturnCode _readSceneInfoFromPhysicsServer();
    void _setJointStates(const ArmConfiguration &joint_states);

    // ___Attributes___
    helpers::Watchdog::SharedPtr _watchdog;
    rclcpp_action::Server<GeneratePathPose>::SharedPtr _action_server_pose;
    rclcpp::Publisher<custom_interfaces::msg::GeneratedPath>::SharedPtr _generated_path_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_state_sub;
    std::mutex _current_joint_states_mtx;
    sensor_msgs::msg::JointState::SharedPtr _current_joint_states;
    helpers::commons::RobotInfo _robot_info;

    SceneInfo::SharedPtr _scene_info;

    /**
     * @brief ID in physics server of table and all static things which are not changing e.g. camera stands, artificial walls for collisions
     * The reason to keep it separately and not as an generic obstacle is to easily keep track of changing obstacles when new moving items
     * appear on the table.
     */
    int _table_idx;

    const float _safety_range = 0.003;
    const int _contact_number_allowed = 1;
  };

} // namespace generate_path

#endif // GENERATE_PATH__GENERATE_PATH_HPP_
