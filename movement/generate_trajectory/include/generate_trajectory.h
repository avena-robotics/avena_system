#pragma once

#include "path.h"
#include "trajectory.h"
#include <iostream>
#include <rclcpp/time.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <custom_interfaces/action/simple_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "rclcpp_components/register_node_macro.hpp"

#include <helpers_commons/helpers_commons.hpp>
#include "visibility_control.h"

namespace generate_trajectory
{

    class GenerateTrajectory : public rclcpp::Node
    {
    public:
        GENERATE_TRAJECTORY_PUBLIC
        explicit GenerateTrajectory(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        using GenerateTraj = custom_interfaces::action::SimpleAction;
        using GoalHandleGenerateTraj = rclcpp_action::ServerGoalHandle<GenerateTraj>;

        GENERATE_TRAJECTORY_LOCAL
        bool generateTrajectoryFromPath();

    private:
        GENERATE_TRAJECTORY_LOCAL
        void _generateAcceleration(Eigen::VectorXd maxAcceleration, Trajectory &trajectory);

        rclcpp_action::Server<GenerateTraj>::SharedPtr _action_server_generate;

        GENERATE_TRAJECTORY_LOCAL
        rclcpp_action::GoalResponse _handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GenerateTraj::Goal> goal);
        GENERATE_TRAJECTORY_LOCAL
        rclcpp_action::CancelResponse _handleCancel(const std::shared_ptr<GoalHandleGenerateTraj> goal_handle);
        GENERATE_TRAJECTORY_LOCAL
        void _handleAccepted(const std::shared_ptr<GoalHandleGenerateTraj> goal_handle);
        GENERATE_TRAJECTORY_LOCAL
        void _execute(const std::shared_ptr<GoalHandleGenerateTraj> goal_handle);

        GENERATE_TRAJECTORY_LOCAL
        const Path _convertTrajMsgToPath(trajectory_msgs::msg::JointTrajectory::SharedPtr path_data);
        GENERATE_TRAJECTORY_LOCAL
        void _convertToMsgAndPublishTraj(Trajectory &trajectory);

        GENERATE_TRAJECTORY_LOCAL
        int _getParametersFromServer();

        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr _sub_path;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _pub_trajectory;
        trajectory_msgs::msg::JointTrajectory::SharedPtr _path_data;

        const double _time_step = 0.033;

        bool _parameters_read;
        helpers::commons::RobotInfo _robot_info;
    };

}