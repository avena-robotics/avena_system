#ifndef PATH_BUFFER_HPP_
#define PATH_BUFFER_HPP_
// __CPP__
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// ___Avena___
#include "custom_interfaces/action/path_buffer.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "visibility_control.h"
#include "helpers_commons/helpers_commons.hpp"

namespace path_buffer
{
    class PathBuffer : public rclcpp::Node
    {
    public:
        // Action
        using PathBufferAction = custom_interfaces::action::PathBuffer;
        using GoalHandlePathBuffer = rclcpp_action::ServerGoalHandle<PathBufferAction>;

        PATH_BUFFER_PUBLIC
        explicit PathBuffer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        // helpers::Watchdog::SharedPtr _watchdog;

        std::map<std::string, trajectory_msgs::msg::JointTrajectory> _paths;
        // ROS
        // Actions
        PATH_BUFFER_LOCAL
        rclcpp_action::GoalResponse _handle_goal_get(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const PathBufferAction::Goal> goal);
        PATH_BUFFER_LOCAL
        rclcpp_action::CancelResponse _handle_cancel_get(const std::shared_ptr<GoalHandlePathBuffer> goal_handle);
        PATH_BUFFER_LOCAL
        void _handle_accepted_get(const std::shared_ptr<GoalHandlePathBuffer> goal_handle);
        PATH_BUFFER_LOCAL
        rclcpp_action::GoalResponse _handle_goal_set(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const PathBufferAction::Goal> goal);
        PATH_BUFFER_LOCAL
        rclcpp_action::CancelResponse _handle_cancel_set(const std::shared_ptr<GoalHandlePathBuffer> goal_handle);
        PATH_BUFFER_LOCAL
        void _handle_accepted_set(const std::shared_ptr<GoalHandlePathBuffer> goal_handle);
        PATH_BUFFER_LOCAL
        int _validateInput();
        rclcpp_action::Server<PathBufferAction>::SharedPtr _action_server_get;
        rclcpp_action::Server<PathBufferAction>::SharedPtr _action_server_set;
        // Publisher
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _pub_path;
        // Subscriber
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr _sub_path;
        trajectory_msgs::msg::JointTrajectory::SharedPtr _input_msg_data;
    };
} // namespace path_buffer

#endif //PATH_BUFFER_HPP_
