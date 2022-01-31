// CPP
#include <chrono>
#include <memory>
#include <filesystem>
#include <fstream>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

// constexpr int JOINTS = 6;
constexpr double JOINT_POSMAX = M_PI;
constexpr double JOINT_VELMAX = 2 * M_PI;
constexpr double JOINT_ACCMAX = 4 * M_PI;
constexpr double JOINT_TORQUEMAX = 256.0;
constexpr double MAX_INT16 = 32767.0;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("play_generate_trajectories");
using Action = control_msgs::action::FollowJointTrajectory;

namespace fs = std::filesystem;

std::vector<std::string> tokenize(const std::string &s, const std::string &del = " ")
{
    int start = 0;
    int end = s.find(del);
    std::vector<std::string> tokens;
    while (end != -1)
    {
        tokens.push_back(s.substr(start, end - start));
        start = end + del.size();
        end = s.find(del, start);
    }
    tokens.push_back(s.substr(start, end - start));
    return tokens;
}

std::tuple<std::vector<double>, size_t> convertStrLineToDoublesVector(const std::string &line)
{
    const auto tokens = tokenize(line, ",");
    std::vector<double> values;
    const size_t nr_joints = (tokens.size() - 1) / 3;
    size_t idx = 0;
    values.push_back(static_cast<double>(std::stoi(tokens[idx++])));
    for (size_t i = 0; i < nr_joints; i++)
        values.push_back(static_cast<double>(std::stoi(tokens[idx++])) / MAX_INT16 * JOINT_POSMAX);
    for (size_t i = 0; i < nr_joints; i++)
        values.push_back(static_cast<double>(std::stoi(tokens[idx++])) / MAX_INT16 * JOINT_VELMAX);
    for (size_t i = 0; i < nr_joints; i++)
        values.push_back(static_cast<double>(std::stoi(tokens[idx++])) / MAX_INT16 * JOINT_ACCMAX);
    return {values, nr_joints};
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opt;
    opt.allow_undeclared_parameters(true);
    opt.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("play_generate_trajectories", opt);
    
    bool found_rviz = false;
    while (!found_rviz)
    {
        auto node_names = node->get_node_names();
        for (auto n : node_names)
        {
            // FIXME: This is a hack to make sure that RViz is running
            if (n.find("rviz2_private") != std::string::npos)
            {
                found_rviz = true;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(LOGGER, "Playing generated trajectories");

    auto action_client = rclcpp_action::create_client<Action>(node, "avena_arm_controller/follow_joint_trajectory");
    if (!action_client->wait_for_action_server(std::chrono::seconds(20)))
    {
        RCLCPP_ERROR(LOGGER, "Action server not available after waiting");
        return 1;
    }

    fs::path base_path;
    if (node->get_parameter("base_path", base_path))
    {
        RCLCPP_INFO_STREAM(LOGGER, "Generated trajectories will be read from directory: " << base_path);
    }
    else
    {
        base_path = "/home/avena/Documents";
        RCLCPP_WARN_STREAM(LOGGER, "Parameter \"base_path\" was not specified. Reading generated trajectories from " << base_path);
    }

    std::vector<std::filesystem::path> trajectories_files_paths;
    for (const auto &file : fs::directory_iterator(base_path))
        if (file.path().extension() == ".csv")
            trajectories_files_paths.push_back(file.path());

    // std::filesystem::directory_iterator does not iterate in order, no names 
    // have to sorted assuming that names of CSV files can be sorted
    std::sort(trajectories_files_paths.begin(), trajectories_files_paths.end());

    for (const auto &file_path : trajectories_files_paths)
    {
        RCLCPP_INFO_STREAM(LOGGER, "Playing trajectory from path " << file_path);

        trajectory_msgs::msg::JointTrajectory trajectory;
        trajectory.header.stamp = node->get_clock()->now();

        std::ifstream csv_file(file_path);
        if (csv_file.is_open())
        {
            std::string line;
            while (std::getline(csv_file, line))
            {
                const auto [values, nr_joints] = convertStrLineToDoublesVector(line);
                const auto time_from_start_ms = values[0];
                trajectory_msgs::msg::JointTrajectoryPoint jtp;
                for (size_t i = 0; i < nr_joints; i++)
                {
                    jtp.positions.push_back(values[i + 1]);
                    jtp.velocities.push_back(values[i + 1 + nr_joints]);
                    jtp.accelerations.push_back(values[i + 1 + nr_joints * 2]);
                }
                jtp.time_from_start.sec = static_cast<int32_t>(time_from_start_ms / 1000);
                jtp.time_from_start.nanosec = static_cast<uint32_t>((time_from_start_ms - jtp.time_from_start.sec * 1000) * 1e6);
                trajectory.points.push_back(jtp);
            }
        }
        csv_file.close();

        // Populate a goal
        auto goal_msg = Action::Goal();
        goal_msg.trajectory = trajectory;

        // RCLCPP_INFO(LOGGER, "Sending goal");
        // Ask server to achieve some goal and wait until it's accepted
        auto goal_handle_future = action_client->async_send_goal(goal_msg);
        if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(LOGGER, "send goal call failed :(");
            continue;
        }

        rclcpp_action::ClientGoalHandle<Action>::SharedPtr goal_handle = goal_handle_future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
            continue;
        }

        // Wait for the server to be done with the goal
        auto result_future = action_client->async_get_result(goal_handle);

        RCLCPP_INFO(LOGGER, "Waiting for result");
        if (rclcpp::spin_until_future_complete(node, result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(LOGGER, "get result call failed :(");
            continue;
        }

        rclcpp_action::ClientGoalHandle<Action>::WrappedResult wrapped_result = result_future.get();

        switch (wrapped_result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO_STREAM(LOGGER, "Success");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(LOGGER, "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(LOGGER, "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(LOGGER, "Unknown result code");
            break;
        }

        RCLCPP_INFO_STREAM(LOGGER, "Result received. Error code: " << wrapped_result.result->error_code);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    rclcpp::shutdown();
    return 0;
}
