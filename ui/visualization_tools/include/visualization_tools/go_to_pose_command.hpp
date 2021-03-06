#ifndef VISUALIZATION_TOOLS__GO_TO_POSE_COMMAND_HPP_
#define VISUALIZATION_TOOLS__GO_TO_POSE_COMMAND_HPP_

#include <Eigen/Dense>

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// ___Avena___
#include <custom_interfaces/srv/data_store_movement_sequence_insert.hpp>
#include <custom_interfaces/srv/data_store_scene_insert.hpp>
#include <custom_interfaces/action/simple_action.hpp>
#include <helpers_vision/helpers_vision.hpp>
#include <helpers_commons/helpers_commons.hpp>

namespace visualization_tools
{
  enum class MenuEntries_e
  {
    PATH,
    LINEAR_PATH,
    ORIENTATION_PATH,
    LO_PATH,
    CLEAR_SEQUENCE,
    START_PLANNING,
  };

  class GoToPoseCommand : public rclcpp::Node
  {
  public:
    using MovementSequenceInsert = custom_interfaces::srv::DataStoreMovementSequenceInsert;
    using OctomapInsert = custom_interfaces::srv::DataStoreSceneInsert;

    using EndEffectorPose = custom_interfaces::msg::EndEffectorPose;

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
    // void _drawConstraintArea(const geometry_msgs::msg::Pose &start_end_effector_pose, const geometry_msgs::msg::Pose &requested_end_effector_pose);
    visualization_msgs::msg::Marker _getConstraintLine(const geometry_msgs::msg::Pose &start_end_effector_pose, const geometry_msgs::msg::Pose &requested_end_effector_pose);
    void _deleteMotionPlanningInfoMarkers();

    void _writeMovementSequenceToServer() noexcept(false);
    void _saveSequenceSegmentToBuffer(const MenuEntries_e &path_type) noexcept(false);
    // void _writeSceneOctomapToServer() noexcept(false);

    // Generate path
    void _sendGeneratePathGoal(const geometry_msgs::msg::Pose &requested_end_effector_pose);

    // Generate trajectory
    void _sendGenerateTrajectoryGoal();

    // Execute move
    void _sendExecuteMoveGoal();
    void _resultCallback(const rclcpp_action::ResultCode &result_code, const std::string &message);

    std::shared_ptr<interactive_markers::InteractiveMarkerServer> _interactive_markers_server;
    interactive_markers::MenuHandler _menu_handler;
    std::map<interactive_markers::MenuHandler::EntryHandle, MenuEntries_e> _menu_entries;

    helpers::commons::RobotInfo _robot_info;
    geometry_msgs::msg::Pose _request_go_to_pose;
    std::map<std::string, rclcpp_action::ClientBase::SharedPtr> _movement_action_clients;
    rclcpp::Client<MovementSequenceInsert>::SharedPtr _movement_sequence_insert_client;
    rclcpp::Client<OctomapInsert>::SharedPtr _octomap_insert_client;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _motion_planning_info_markers_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _sequence_poses_pub;
    size_t _marker_count;
    std::vector<EndEffectorPose> _sequence_to_execute;

    // bool _lock_marker_interaction;

    // ___Constants___
    const std::string GENERATE_TRAJECTORY_NAME = "generate_trajectory";
    // const std::string PATH = "path";
    // const std::string LINEAR_PATH = "linear_path";

    // // const std::string EXECUTE_MOVE_NAME = "execute_move";
    // const std::string RESET_MARKER_POSE = "reset_marker_pose";
    const std::chrono::seconds WAITING_FOR_ACTION_TIMEOUT = std::chrono::seconds(2);
  };
} // namespace visualization_tools

#endif // VISUALIZATION_TOOLS__GO_TO_POSE_COMMAND_HPP_