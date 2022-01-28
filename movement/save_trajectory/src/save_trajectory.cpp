#include <save_trajectory/save_trajectory.hpp>

namespace save_trajectory
{
    SaveTrajectory::SaveTrajectory(rclcpp::NodeOptions options)
        : Node("avena_arm_controller", options
                                           .allow_undeclared_parameters(true)
                                           .automatically_declare_parameters_from_overrides(true))
    {
        RCLCPP_INFO(LOGGER, "Initializing module to save generated trajectories to file and/or database");

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

        const auto qos_latching = rclcpp::QoS(1).transient_local().reliable();
        _trajectory_sub = create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/debug/trajectory", qos_latching, 
            std::bind(&SaveTrajectory::_trajectoryCallback, this, std::placeholders::_1));

    }

    void SaveTrajectory::_trajectoryCallback(trajectory_msgs::msg::JointTrajectory::ConstSharedPtr trajectory)
    {
        RCLCPP_INFO(LOGGER, "Saving generated trajectory to file");
        // const auto goal = goal_handle->get_goal();
        RCLCPP_INFO_STREAM(LOGGER, "Number of points in trajectory " << trajectory->points.size());

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
        for (size_t idx = 0; idx < trajectory->points.size(); idx++)
        {
            const auto &jtp = trajectory->points[idx];
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
            if (idx < trajectory->points.size() - 1)
                data_stream << std::endl;
        }

        const auto file_path = _base_path / (std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".csv");
        std::ofstream f(file_path);
        f << data_stream.rdbuf();
        f.close();

        RCLCPP_INFO_STREAM(LOGGER, "Trajectory saved to " << file_path);
    }
} // namespace save_trajectory

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(save_trajectory::SaveTrajectory)
