#include "helpers_commons/robot_utils.hpp"

namespace helpers
{
    namespace commons
    {
        std::string getRobotDescription(rclcpp::Node *node)
        {
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, "robot_state_publisher");
            while (!parameters_client->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("helpers"), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                    return {};
                }
                RCLCPP_INFO(rclcpp::get_logger("helpers"), "service not available, waiting again...");
            }
            std::string robot_urdf = parameters_client->get_parameter<std::string>("robot_description");
            return robot_urdf;
        }

        std::vector<std::string> getRobotLinksNames(rclcpp::Node *node)
        {
            std::string robot_urdf = getRobotDescription(node);
            urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(robot_urdf);
            std::vector<std::string> link_names;
            for (auto &[link_name, link_info] : model->links_)
            {
                if (link_info->visual || link_info->visual_array.size() != 0)
                    link_names.push_back(link_name);
            }
            return link_names;
        }

        std::vector<std::string> getRobotLinksNames(rclcpp::Node::SharedPtr node)
        {
            return getRobotLinksNames(node.get());
        }

        RobotInfo getRobotInfo(const std::string &side)
        {
            // TODO: Most of stuff in this function should be read from URDF
            RobotInfo robot_info;
            if (side == "right") // Franka
            {
                robot_info.robot_name = "franka";
                robot_info.robot_prefix = "right_" + robot_info.robot_name;
                robot_info.connection = robot_info.robot_prefix + "_gripper_connection";
                for (size_t idx = 0; idx <= 7; ++idx)
                    robot_info.link_names.push_back(robot_info.robot_prefix + "_link_" + std::to_string(idx));
                robot_info.nr_links = robot_info.link_names.size();

                for (size_t idx = 1; idx <= 7; ++idx)
                    robot_info.joint_names.push_back(robot_info.robot_prefix + "_joint_" + std::to_string(idx));
                robot_info.nr_joints = robot_info.joint_names.size();

                // Joints bounds
                const float joint_bounds_coeff = 0.98;
                robot_info.bounds.resize(robot_info.nr_joints);
                robot_info.bounds[0].bounds_low = -2.8973 * joint_bounds_coeff;
                robot_info.bounds[0].bounds_high = 2.8973 * joint_bounds_coeff;

                robot_info.bounds[1].bounds_low = -1.7628 * joint_bounds_coeff;
                robot_info.bounds[1].bounds_high = 1.7628 * joint_bounds_coeff;

                robot_info.bounds[2].bounds_low = -2.8973 * joint_bounds_coeff;
                robot_info.bounds[2].bounds_high = 2.8973 * joint_bounds_coeff;

                robot_info.bounds[3].bounds_low = -3.0718 * joint_bounds_coeff;
                robot_info.bounds[3].bounds_high = -0.0698 * joint_bounds_coeff;

                robot_info.bounds[4].bounds_low = -2.8973 * joint_bounds_coeff;
                robot_info.bounds[4].bounds_high = 2.8973 * joint_bounds_coeff;

                robot_info.bounds[5].bounds_low = -0.0175 * joint_bounds_coeff;
                robot_info.bounds[5].bounds_high = 3.7525 * joint_bounds_coeff;

                robot_info.bounds[6].bounds_low = -2.8973 * joint_bounds_coeff;
                robot_info.bounds[6].bounds_high = 2.8973 * joint_bounds_coeff;
            }
            else if (side == "left")
            {
                robot_info.robot_name = "avena";
                robot_info.robot_prefix = "left_" + robot_info.robot_name;
                robot_info.connection = robot_info.robot_prefix + "_gripper_connection";
                for (size_t idx = 0; idx <= 6; ++idx)
                    robot_info.link_names.push_back(robot_info.robot_prefix + "_link_" + std::to_string(idx));
                robot_info.nr_links = robot_info.link_names.size();

                for (size_t idx = 1; idx <= 6; ++idx)
                    robot_info.joint_names.push_back(robot_info.robot_prefix + "_joint_" + std::to_string(idx));
                robot_info.nr_joints = robot_info.joint_names.size();

                // Joints bounds
                robot_info.bounds.resize(robot_info.nr_joints);
                robot_info.bounds[0].bounds_low = -3.05;
                robot_info.bounds[0].bounds_high = 3.05;

                robot_info.bounds[1].bounds_low = -1.57628;
                robot_info.bounds[1].bounds_high = 1.57628;

                robot_info.bounds[2].bounds_low = -2.8973;
                robot_info.bounds[2].bounds_high = 2.8973;

                robot_info.bounds[3].bounds_low = -3.05;
                robot_info.bounds[3].bounds_high = 3.05;

                robot_info.bounds[4].bounds_low = -3.05;
                robot_info.bounds[4].bounds_high = 3.05;

                robot_info.bounds[5].bounds_low = -3.05;
                robot_info.bounds[5].bounds_high = 3.05;
            }
            return robot_info;
        }

    } // namespace commons
} // namespace helpers
