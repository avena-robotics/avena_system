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
    void _processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
    visualization_msgs::msg::Marker _makeBox(const visualization_msgs::msg::InteractiveMarker &msg);
    visualization_msgs::msg::Marker _makeSphere(const visualization_msgs::msg::InteractiveMarker &msg);
    void _createGoToPoseCommandMarker();
    void _setupMenuEntries(interactive_markers::MenuHandler &menu_handler);
    geometry_msgs::msg::Quaternion _eigenToRos(const Eigen::Quaternionf &orien);

    // Generate path
    void _sendGeneratePathGoal(const geometry_msgs::msg::Pose &requested_end_effector_pose);
    void _goalResponseGeneratePathCallback(std::shared_future<GoalHandleGeneratePathPose::SharedPtr> /*future*/){};
    void _feedbackGeneratePathCallback(GoalHandleGeneratePathPose::SharedPtr, const std::shared_ptr<const GeneratePathPose::Feedback> /*feedback*/){};
    void _resultGeneratePathCallback(const GoalHandleGeneratePathPose::WrappedResult &result);

    // // Generate path pick
    // void _sendGeneratePathPickGoal(const geometry_msgs::msg::Pose &requested_end_effector_pose);
    // void _goalResponseGeneratePathPickCallback(std::shared_future<GoalHandleGeneratePathPick::SharedPtr> /*future*/){};
    // void _feedbackGeneratePathPickCallback(GoalHandleGeneratePathPick::SharedPtr, const std::shared_ptr<const GeneratePathPick::Feedback> /*feedback*/){};
    // void _resultGeneratePathPickCallback(const GoalHandleGeneratePathPick::WrappedResult &result);

    // Generate trajectory
    void _sendGenerateTrajectoryGoal();
    void _goalResponseGenerateTrajectoryCallback(std::shared_future<GoalHandleGenerateTrajectory::SharedPtr> /*future*/){};
    void _feedbackGenerateTrajectoryCallback(GoalHandleGenerateTrajectory::SharedPtr, const std::shared_ptr<const GenerateTrajectory::Feedback> /*feedback*/){};
    void _resultGenerateTrajectoryCallback(const GoalHandleGenerateTrajectory::WrappedResult &result);

    // Execute move
    void _sendExecuteMoveGoal();
    void _goalResponseExecuteMoveCallback(std::shared_future<GoalHandleExecuteMove::SharedPtr> /*future*/){};
    void _feedbackExecuteMoveCallback(GoalHandleExecuteMove::SharedPtr, const std::shared_ptr<const ExecuteMove::Feedback> /*feedback*/){};
    void _resultExecuteMoveCallback(const GoalHandleExecuteMove::WrappedResult &result);

    void _resultCallback(const rclcpp_action::ResultCode &result_code, const std::string &message);

    std::shared_ptr<interactive_markers::InteractiveMarkerServer> _interactive_markers_server;
    interactive_markers::MenuHandler _menu_handler;
    std::map<interactive_markers::MenuHandler::EntryHandle, std::string> _entries_to_action_name;

    geometry_msgs::msg::Pose _request_go_to_pose;
    rclcpp_action::Client<GeneratePathPose>::SharedPtr _generate_path_pose_client;
    // rclcpp_action::Client<GeneratePathPick>::SharedPtr _generate_path_pick_client;
    rclcpp_action::Client<GenerateTrajectory>::SharedPtr _generate_trajectory_client;
    rclcpp_action::Client<ExecuteMove>::SharedPtr _execute_move_client;
    bool _lock_marker_interaction;

    const std::string GENERATE_PATH_NAME = "generate_path_pose";
    const std::string GENERATE_TRAJECTORY_NAME = "generate_trajectory";
    // const std::string GENERATE_PATH_PICK_NAME = "generate_path_pick";
    // const std::string GENERATE_PATH_PLACE_NAME = "generate_path_place";
    const std::string EXECUTE_MOVE_NAME = "execute_move";
    const std::chrono::seconds WAITING_FOR_ACTION_TIMEOUT = std::chrono::seconds(2);
  };
} // namespace visualization_tools

#endif // VISUALIZATION_TOOLS__Go_TO_POSE_COMMAND_HPP_