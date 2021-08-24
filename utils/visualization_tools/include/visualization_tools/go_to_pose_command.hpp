#ifndef VISUALIZATION_TOOLS__Go_TO_POSE_COMMAND_HPP_
#define VISUALIZATION_TOOLS__Go_TO_POSE_COMMAND_HPP_

#include <Eigen/Dense>

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

// ___Avena___
#include <custom_interfaces/action/generate_path_pose.hpp>
#include <custom_interfaces/action/simple_action.hpp>
// #include <custom_interfaces/action/generate_path_pick_action.hpp>
#include <helpers_vision/helpers_vision.hpp>
#include <helpers_commons/helpers_commons.hpp>

namespace visualization_tools
{
  class GoToPoseCommand : public rclcpp::Node
  {
  public:
    using GeneratePathPose = custom_interfaces::action::GeneratePathPose;
    using GoalHandleGeneratePathPose = rclcpp_action::ClientGoalHandle<GeneratePathPose>;

    // using GeneratePathPick = custom_interfaces::action::GeneratePathPickAction;
    // using GoalHandleGeneratePathPick = rclcpp_action::ClientGoalHandle<GeneratePathPick>;

    using GenerateTrajectory = custom_interfaces::action::SimpleAction;
    using GoalHandleGenerateTrajectory = rclcpp_action::ClientGoalHandle<GenerateTrajectory>;

    using ExecuteMove = custom_interfaces::action::SimpleAction;
    using GoalHandleExecuteMove = rclcpp_action::ClientGoalHandle<ExecuteMove>;

    explicit GoToPoseCommand(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~GoToPoseCommand() = default;

  private:
    void _markerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
    void _movementFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
    void _resetMarkerPoseFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
    visualization_msgs::msg::Marker _makeBox(const visualization_msgs::msg::InteractiveMarker &msg);
    visualization_msgs::msg::Marker _makeSphere(const visualization_msgs::msg::InteractiveMarker &msg);
    void _createGoToPoseCommandMarker();
    void _setupMenuEntries(interactive_markers::MenuHandler &menu_handler);
    geometry_msgs::msg::Quaternion _eigenToRos(const Eigen::Quaternionf &orien);
    std::optional<geometry_msgs::msg::Pose> _getEndEffectorPose();

    // Generate path
    void _sendGeneratePathGoal(const geometry_msgs::msg::Pose &requested_end_effector_pose);

    // // Generate path pick
    // void _sendGeneratePathPickGoal(const geometry_msgs::msg::Pose &requested_end_effector_pose);

    // Generate trajectory
    void _sendGenerateTrajectoryGoal();

    // Execute move
    void _sendExecuteMoveGoal();

    void _resultCallback(const rclcpp_action::ResultCode &result_code, const std::string &message);

    std::shared_ptr<interactive_markers::InteractiveMarkerServer> _interactive_markers_server;
    interactive_markers::MenuHandler _menu_handler;
    std::map<interactive_markers::MenuHandler::EntryHandle, std::string> _menu_entries;

    helpers::commons::RobotInfo _robot_info;
    geometry_msgs::msg::Pose _request_go_to_pose;
    std::map<std::string, rclcpp_action::ClientBase::SharedPtr> _movement_action_clients;

    // bool _lock_marker_interaction;

    // ___Constants___
    const std::string GENERATE_PATH_NAME = "generate_path_pose";
    const std::string GENERATE_TRAJECTORY_NAME = "generate_trajectory";
    // const std::string GENERATE_PATH_PICK_NAME = "generate_path_pick";
    // const std::string GENERATE_PATH_PLACE_NAME = "generate_path_place";
    const std::string EXECUTE_MOVE_NAME = "execute_move";
    const std::string RESET_MARKER_POSE = "reset_marker_pose";
    const std::chrono::seconds WAITING_FOR_ACTION_TIMEOUT = std::chrono::seconds(2);
  };
} // namespace visualization_tools

#endif // VISUALIZATION_TOOLS__Go_TO_POSE_COMMAND_HPP_