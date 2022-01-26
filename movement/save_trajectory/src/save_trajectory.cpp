// CPP
#include <memory>
#include <fstream>

// ROS2
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

constexpr double JOINT_POSMAX = M_PI;
constexpr double JOINT_VELMAX = 2 * M_PI;
constexpr double JOINT_ACCMAX = 4 * M_PI;
constexpr double JOINT_TORQUEMAX = 256.0;
constexpr double MAX_INT16 = 32767.0;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("avena_arm_saving_trajectory");

using Action = control_msgs::action::FollowJointTrajectory;
using GoalHandleAction = rclcpp_action::ServerGoalHandle<Action>;

rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const Action::Goal> /*goal*/)
{
    RCLCPP_INFO(LOGGER, "Got goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAction> /*goal_handle*/)
{
    RCLCPP_INFO(LOGGER, "Got request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void execute(const std::shared_ptr<GoalHandleAction> goal_handle)
{
    RCLCPP_INFO(LOGGER, "Saving generated trajectory to file");
    const auto goal = goal_handle->get_goal();

    // ////////////////////////////////////////////////////////////
    // std::stringstream ss;
    // ss << "\n---\nJoint names:" << std::endl;
    // for (auto jn : goal->trajectory.joint_names)
    //     ss << "\t- " << jn << std::endl;

    // for (size_t idx = 0; idx < goal->trajectory.points.size(); idx++)
    // {
    //     const auto &jtp = goal->trajectory.points[idx];
    //     ss << "Idx: " << idx << " -> time from start: " << jtp.time_from_start.sec + jtp.time_from_start.nanosec / 1e9 << " sec, joint values: ";
    //     for (const auto val : jtp.positions)
    //         ss << val << " ";
    //     ss << std::endl;
    // }
    // RCLCPP_INFO(LOGGER, ss.str());
    // ////////////////////////////////////////////////////////////

    std::stringstream data_stream;
    for (size_t idx = 0; idx < goal->trajectory.points.size(); idx++)
    {
        const auto &jtp = goal->trajectory.points[idx];
        int time_from_start_ms = static_cast<int>((jtp.time_from_start.sec + jtp.time_from_start.nanosec / 1e9) * 1000);
        data_stream << time_from_start_ms << ",";
        for (const auto pos : jtp.positions)
            data_stream << static_cast<int>(pos * MAX_INT16 / JOINT_POSMAX) << ",";

        for (const auto vel : jtp.velocities)
            data_stream << static_cast<int>(vel * MAX_INT16 / JOINT_VELMAX) << ",";
        
        for (size_t joint_idx = 0; joint_idx < jtp.accelerations.size(); joint_idx++)
        {
            data_stream << static_cast<int>(jtp.accelerations[joint_idx] * MAX_INT16 / JOINT_ACCMAX);
            if (joint_idx < jtp.accelerations.size() - 1)
                data_stream << ",";
        }
        if (idx < goal->trajectory.points.size() - 1)
            data_stream << std::endl;
    }

    const std::string file_path = "/home/avena/ros2_ws/src/TrajectoryInt.csv";
    std::ofstream f(file_path);
    f << data_stream.rdbuf();
    f.close();

    // Check if goal is done
    if (rclcpp::ok())
    {
        auto result = std::make_shared<Action::Result>();
        goal_handle->succeed(result);
        RCLCPP_INFO_STREAM(LOGGER, "Goal Succeeded - trajectory saved to \"" << file_path << "\"");
    }
}

void handle_accepted(const std::shared_ptr<GoalHandleAction> goal_handle)
{
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{execute, goal_handle}.detach();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("avena_arm_controller");

    // Create an action server with three callbacks
    //   'handle_goal' and 'handle_cancel' are called by the Executor (rclcpp::spin)
    //   'execute' is called whenever 'handle_goal' returns by accepting a goal
    //    Calls to 'execute' are made in an available thread from a pool of four.
    auto action_server = rclcpp_action::create_server<Action>(
        node,
        "avena_arm_controller/follow_joint_trajectory",
        handle_goal,
        handle_cancel,
        handle_accepted);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}