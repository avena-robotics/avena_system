#pragma once

#include "path.h"
#include "trajectory.h"
#include <iostream>
#include <rclcpp/time.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <custom_interfaces/action/simple_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <custom_interfaces/msg/generated_path.hpp>
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
        using GeneratedPath = custom_interfaces::msg::GeneratedPath;

    private:
        GENERATE_TRAJECTORY_LOCAL
        int _generateTrajectoryFromPathSegment(trajectory_msgs::msg::JointTrajectory &segment_trajectory, trajectory_msgs::msg::JointTrajectory &out_trajectory);
        
        GENERATE_TRAJECTORY_LOCAL
        std::unique_ptr<trajectory_msgs::msg::JointTrajectory> _generateTrajectoryFromPath(const GeneratedPath::SharedPtr &generated_path);
        
        GENERATE_TRAJECTORY_LOCAL
        void _generateAcceleration(Eigen::VectorXd maxAcceleration, Trajectory &trajectory);


        GENERATE_TRAJECTORY_LOCAL
        rclcpp_action::GoalResponse _handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GenerateTraj::Goal> goal);

        GENERATE_TRAJECTORY_LOCAL
        rclcpp_action::CancelResponse _handleCancel(const std::shared_ptr<GoalHandleGenerateTraj> goal_handle);

        GENERATE_TRAJECTORY_LOCAL
        void _handleAccepted(const std::shared_ptr<GoalHandleGenerateTraj> goal_handle);

        GENERATE_TRAJECTORY_LOCAL
        void _execute(const std::shared_ptr<GoalHandleGenerateTraj> goal_handle);

        // GENERATE_TRAJECTORY_LOCAL
        // const Path _convertTrajMsgToPath(trajectory_msgs::msg::JointTrajectory::SharedPtr path_data);

        GENERATE_TRAJECTORY_LOCAL
        void _convertToMsg(Trajectory &trajectory, const std::vector<std::string> &joint_names, trajectory_msgs::msg::JointTrajectory &out_trajectory);

        GENERATE_TRAJECTORY_LOCAL
        int _getParametersFromServer();

        rclcpp_action::Server<GenerateTraj>::SharedPtr _action_server_generate;
        rclcpp::Subscription<GeneratedPath>::SharedPtr _sub_path;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _pub_trajectory;
        GeneratedPath::SharedPtr _path_data;

        double _time_step;
        std::vector<std::string> _joint_names;
        bool _parameters_read;
        helpers::commons::RobotInfo _robot_info;

    };

}