#include <save_trajectory/save_trajectory.hpp>

namespace save_trajectory
{
    SaveTrajectory::SaveTrajectory(rclcpp::NodeOptions options)
        : Node("avena_arm_controller", options
                                           .allow_undeclared_parameters(true)
                                           .automatically_declare_parameters_from_overrides(true))
    {
        RCLCPP_INFO(LOGGER, "Initializing dummy joint trajectory to save generated trajectories to file and/or database");

        get_parameter("run_joint_state_pub", _run_joint_state_pub);
        if (_run_joint_state_pub)
        {
            RCLCPP_INFO_STREAM(LOGGER, "Running dummy joint state publisher");
        }
        else
        {
            RCLCPP_WARN_STREAM(LOGGER, "Other module provides current joint states for motion planning");
        }

        if (get_parameter("base_path", _base_path))
        {
            // _base_path = std::filesystem::path(base_path_str);
            RCLCPP_INFO_STREAM(LOGGER, "Generated trajectories will be saved to path: " << _base_path);
        }
        else
        {
            _base_path = "/home/avena/Documents";
            RCLCPP_WARN_STREAM(LOGGER, "Parameter \"base_path\" was not specified. Saving generated trajectories to " << _base_path);
        }

        _action_server = rclcpp_action::create_server<Action>(
            get_node_base_interface(),
            get_node_clock_interface(),
            get_node_logging_interface(),
            get_node_waitables_interface(),
            "avena_arm_controller/follow_joint_trajectory",
            std::bind(&SaveTrajectory::_handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&SaveTrajectory::_handleCancel, this, std::placeholders::_1),
            std::bind(&SaveTrajectory::_handleAccepted, this, std::placeholders::_1));

        if (_run_joint_state_pub)
        {
            _dummy_joint_state_pub = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(10).reliable());
            _current_joint_states.name.clear();
            for (size_t i = 0; i < 6; i++)
            {
                _current_joint_states.name.push_back("avena_joint_" + std::to_string(i + 1));
                _current_joint_states.position.push_back(0);
                _current_joint_states.velocity.push_back(0);
                _current_joint_states.effort.push_back(0);
            }

            _joint_states_timer = create_wall_timer(
                std::chrono::milliseconds(10), [this]()
                {
                    sensor_msgs::msg::JointState joint_states_to_publish;
                    {
                        std::lock_guard<std::mutex> lg(_current_joint_states_mtx);
                        joint_states_to_publish = _current_joint_states;
                    }
                    joint_states_to_publish.header.frame_id = "avena_arm";
                    joint_states_to_publish.header.stamp = get_clock()->now();

                    _dummy_joint_state_pub->publish(joint_states_to_publish);
                });
        }
    }

    rclcpp_action::GoalResponse SaveTrajectory::_handleGoal(const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const Action::Goal> /*goal*/)
    {
        RCLCPP_INFO(LOGGER, "Got goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse SaveTrajectory::_handleCancel(const std::shared_ptr<GoalHandleAction> /*goal_handle*/)
    {
        RCLCPP_INFO(LOGGER, "Got request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void SaveTrajectory::_handleAccepted(const std::shared_ptr<GoalHandleAction> goal_handle)
    {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&SaveTrajectory::_execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void SaveTrajectory::_execute(const std::shared_ptr<GoalHandleAction> goal_handle)
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

        const auto file_path = _base_path / (std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".csv");
        std::ofstream f(file_path);
        f << data_stream.rdbuf();
        f.close();

        // Set current position of arm to last from trajectory
        {
            std::lock_guard<std::mutex> lg(_current_joint_states_mtx);
            const auto &current_jtp = goal->trajectory.points.back();
            _current_joint_states.position = current_jtp.positions;
            _current_joint_states.velocity = current_jtp.velocities;
            _current_joint_states.effort = current_jtp.effort;
        }

        // Check if goal is done
        if (rclcpp::ok())
        {
            auto result = std::make_shared<Action::Result>();
            goal_handle->succeed(result);
            RCLCPP_INFO_STREAM(LOGGER, "Goal Succeeded - trajectory saved to " << file_path);
        }
    }

} // namespace save_trajectory

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(save_trajectory::SaveTrajectory)
