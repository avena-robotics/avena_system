#include "helpers_commons/robot_utils.hpp"

namespace helpers
{
    namespace commons
    {
        std::string getRobotDescription()
        {
            std::string node_name = "reading_robot_description_" + std::to_string(std::rand());
            rclcpp::Node::SharedPtr temp_node = rclcpp::Node::make_shared(node_name, rclcpp::NodeOptions().enable_rosout(false).use_global_arguments(false));
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(temp_node, "robot_state_publisher");
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

        std::vector<std::string> getRobotLinksNames()
        {
            std::string robot_urdf = getRobotDescription();
            urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(robot_urdf);
            std::vector<std::string> link_names;
            for (auto &[link_name, link_info] : model->links_)
            {
                if (link_info->visual || link_info->visual_array.size() != 0)
                    link_names.push_back(link_name);
            }
            return link_names;
        }

        RobotInfo getRobotInfo(const std::string &side)
        {
            RobotInfo robot_info;

            std::string robot_urdf = getRobotDescription();
            urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(robot_urdf);

            robot_info.robot_name = model->getName();

            // Get links
            for (auto &[link_name, link_info] : model->links_)
            {
                if (link_info->visual || link_info->visual_array.size() != 0)
                {
                    if (!link_info->getParent())
                        robot_info.base_link_name = link_name;
                    if (link_name.find("gripper") == std::string::npos && link_name.find("link") != std::string::npos)
                        robot_info.link_names.push_back(link_name);
                    else
                        robot_info.gripper_info.link_names.push_back(link_name);
                }
            }
            robot_info.nr_links = robot_info.link_names.size();

            // Get joints (fixed and moving)
            for (auto &[joint_name, joint_info] : model->joints_)
            {
                if (joint_info->type == urdf::Joint::REVOLUTE)
                {
                    robot_info.joint_names.push_back(joint_name);
                    robot_info.limits.push_back(Limits(joint_info->limits->lower, joint_info->limits->upper));
                    robot_info.soft_limits.push_back(Limits(joint_info->safety->soft_lower_limit, joint_info->safety->soft_upper_limit));
                }
                else if (joint_info->type == urdf::Joint::FIXED)
                {
                    robot_info.fixed_joint_names.push_back(joint_name);
                }
            }
            robot_info.nr_joints = robot_info.joint_names.size();
            robot_info.nr_fixed_joints = robot_info.fixed_joint_names.size();

            if (robot_info.link_names.size() == 0)
                std::runtime_error("URDF is ill formed. There is no links in it. Fix URDF");

            // Extract working side
            std::string working_side;
            if (robot_info.link_names[0].find("left") != std::string::npos)
                working_side = "left";
            else if (robot_info.link_names[0].find("right") != std::string::npos)
                working_side = "right";
            else
                std::runtime_error("URDF is ill formed. There is no working side in links names. Fix URDF");
            robot_info.robot_prefix = working_side + "_" + robot_info.robot_name;
            robot_info.connection = robot_info.robot_prefix + "_gripper_connection";

            return robot_info;
        }

    } // namespace commons
} // namespace helpers
