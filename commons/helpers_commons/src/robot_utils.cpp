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
            if (!parameters_client->wait_for_service(std::chrono::seconds(3)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("helpers"), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                    return {};
                }
                RCLCPP_INFO(rclcpp::get_logger("helpers"), "Could not connect to robot_state_publisher parameters");
                return {};
            }
            std::string robot_urdf = "";
            try
            {
                robot_urdf = parameters_client->get_parameter<std::string>("robot_description");
            }
            catch(const std::runtime_error& e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("helpers"), e.what());
            }
            return robot_urdf;
        }

        std::vector<std::string> getRobotLinksNames()
        {
            std::string robot_urdf = getRobotDescription();
            if (robot_urdf.empty())
                return {};
            urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(robot_urdf);
            std::vector<std::string> link_names;
            for (auto &[link_name, link_info] : model->links_)
            {
                if (link_info->visual || link_info->visual_array.size() != 0)
                    link_names.push_back(link_name);
            }
            return link_names;
        }

        std::optional<RobotInfo> getRobotInfo(const std::string &side)
        {
            RobotInfo robot_info;

            std::string robot_urdf = getRobotDescription();
            if (robot_urdf.empty())
            {
                RCLCPP_ERROR(rclcpp::get_logger("helpers"), "Cannot read robot description");
                return std::nullopt;
            }

            urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(robot_urdf);

            robot_info.robot_name = model->getName();

            // Get links
            for (auto &[link_name, link_info] : model->links_)
            {
                if (link_info->visual || link_info->visual_array.size() != 0)
                {
                    if (!link_info->getParent())
                    {
                        RCLCPP_DEBUG(rclcpp::get_logger("helpers"), "Found base link name: %s", link_name);
                        robot_info.base_link_name = link_name;
                    }
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
                    if (!joint_info->limits)
                    {
                        RCLCPP_ERROR_STREAM(rclcpp::get_logger("helpers"), "There are not joint limits specify for joint \"" << joint_name << "\". Exiting...");
                        return std::nullopt;                     
                    }
                    robot_info.limits.push_back(Limits(joint_info->limits->lower, joint_info->limits->upper));
                    // if (joint_info->safety)
                    //     robot_info.soft_limits.push_back(Limits(joint_info->safety->soft_lower_limit, joint_info->safety->soft_upper_limit));
                    // else
                    //     RCLCPP_DEBUG(rclcpp::get_logger("helpers"), "There not no soft limits in URDF. This values might not be available, so just use them wisely");
                }
                else if (joint_info->type == urdf::Joint::FIXED)
                {
                    robot_info.fixed_joint_names.push_back(joint_name);
                }
            }
            robot_info.nr_joints = robot_info.joint_names.size();
            robot_info.nr_fixed_joints = robot_info.fixed_joint_names.size();

            if (robot_info.link_names.size() == 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("helpers"), "URDF is ill formed. There is no links in it. Fix URDF");
                return std::nullopt;
            }

            // Extract working side
            std::string working_side;
            if (robot_info.link_names[0].find("left") != std::string::npos)
                working_side = "left";
            else if (robot_info.link_names[0].find("right") != std::string::npos)
                working_side = "right";
            else
                throw std::runtime_error("URDF is ill formed. There is no working side in links names. Fix URDF");
            robot_info.robot_prefix = working_side + "_" + robot_info.robot_name;
            robot_info.connection = robot_info.robot_prefix + "_gripper_connection";

            robot_info.reduceSoftJointsRange(0.95);
            return robot_info;
        }

    } // namespace commons
} // namespace helpers
