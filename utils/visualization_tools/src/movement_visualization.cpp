#include "visualization_tools/movement_visualization.hpp"

namespace visualization_tools
{
    MovementVisualization::MovementVisualization(const rclcpp::NodeOptions &options)
        : Node("movement_visualization", options)
    {
        helpers::commons::setLoggerLevel(get_logger(), "debug");
        _getParametersFromServer();

        const std::string urdf_xml = helpers::commons::getRobotDescription(this);
        _joint_state_to_tf.initModel(urdf_xml);

        if (auto world_to_base_link_tf = _getWorldToBaseLinkTf())
        {
            _world_to_base_link_tf = *world_to_base_link_tf;
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Error occured receiving transform to base link. Exiting...");
            rclcpp::shutdown();
            return;
        }

        auto qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        _nav_path_pub = create_publisher<nav_msgs::msg::Path>("nav_path", qos_settings);
        _generated_path_sub = create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "generated_path", qos_settings, std::bind(&MovementVisualization::_callbackGeneratedPath, this, std::placeholders::_1));
        _trajectory_sub = create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "trajectory", qos_settings, std::bind(&MovementVisualization::_callbackTrajectory, this, std::placeholders::_1));

        _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        _static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

    int MovementVisualization::_getParametersFromServer()
    {
        RCLCPP_INFO_ONCE(get_logger(), "Reading parameters from the server");

        while (true)
        {
            nlohmann::json parameters = helpers::commons::getParameter("robot");
            if (parameters.empty())
                continue;
            const std::string working_side = parameters["working_side"];
            _robot_info = helpers::commons::getRobotInfo(working_side);
            _end_effector_name = _robot_info.connection;
            // _end_effector_name = _robot_info.robot_prefix + "_gripper";
            break;
        }

        RCLCPP_INFO(get_logger(), "Parameters read successfully...");
        return 0;
    }

    void MovementVisualization::_callbackGeneratedPath(const trajectory_msgs::msg::JointTrajectory::SharedPtr generated_path)
    {
        helpers::Timer timer(__func__, get_logger());
        RCLCPP_INFO(get_logger(), "Received generated path as trajectory positions");

        const std::vector<std::string> joint_names = generated_path->joint_names;
        std_msgs::msg::Header out_header;
        out_header.frame_id = "world";
        out_header.stamp = generated_path->header.stamp;

        nav_msgs::msg::Path::UniquePtr end_effector_path(new nav_msgs::msg::Path);
        end_effector_path->header = out_header;

        for (size_t i = 0; i < generated_path->points.size(); ++i)
        {
            auto &generated_path_point = generated_path->points[i];
            std::map<std::string, double> joint_positions;
            for (size_t i = 0; i < joint_names.size(); i++)
            {
                joint_positions.insert(std::make_pair(joint_names[i], generated_path_point.positions[i]));
            }

            auto moving_transforms = _joint_state_to_tf.getMovingTransforms(joint_positions, out_header.stamp);
            auto fixed_transforms = _joint_state_to_tf.getFixedTransforms();
            fixed_transforms.push_back(_world_to_base_link_tf);

            // End effector pose in "world" frame
            auto end_effector_transform = _getEndEffectorTransform(fixed_transforms, moving_transforms);
            geometry_msgs::msg::PoseStamped end_effector_pose;
            end_effector_pose.header = end_effector_transform.header;
            end_effector_pose.pose.position.x = end_effector_transform.transform.translation.x;
            end_effector_pose.pose.position.y = end_effector_transform.transform.translation.y;
            end_effector_pose.pose.position.z = end_effector_transform.transform.translation.z;

            end_effector_pose.pose.orientation.x = end_effector_transform.transform.rotation.x;
            end_effector_pose.pose.orientation.y = end_effector_transform.transform.rotation.y;
            end_effector_pose.pose.orientation.z = end_effector_transform.transform.rotation.z;
            end_effector_pose.pose.orientation.w = end_effector_transform.transform.rotation.w;

            end_effector_path->poses.push_back(end_effector_pose);
        }

        _nav_path_pub->publish(std::move(end_effector_path));
    }

    void MovementVisualization::_callbackTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr trajectory)
    {
        auto ros_time_to_chrono = [](const builtin_interfaces::msg::Duration &duration) -> std::chrono::duration<double> {
            return std::chrono::duration<double>(duration.sec + duration.nanosec / 1e9);
        };

        auto update_frame_ids = [](geometry_msgs::msg::TransformStamped &ts) -> void
        {
            if (ts.header.frame_id != "world")
                ts.header.frame_id = "ghost/" + ts.header.frame_id;
            if (ts.child_frame_id != "world")
                ts.child_frame_id = "ghost/" + ts.child_frame_id;
        };

        helpers::Timer timer(__func__, get_logger());
        RCLCPP_INFO(get_logger(), "Received trajectory");

        const std::vector<std::string> joint_names = trajectory->joint_names;
        std_msgs::msg::Header out_header;
        out_header.frame_id = "world";
        out_header.stamp = trajectory->header.stamp;

        auto start_move_time = std::chrono::steady_clock::now();
        for (size_t i = 0; i < trajectory->points.size(); ++i)
        {
            auto &trajectory_point = trajectory->points[i];
            auto time_from_start = ros_time_to_chrono(trajectory_point.time_from_start);

            std::map<std::string, double> joint_positions;
            for (size_t i = 0; i < joint_names.size(); i++)
            {
                joint_positions.insert(std::make_pair(joint_names[i], trajectory_point.positions[i]));
            }

            auto moving_transforms = _joint_state_to_tf.getMovingTransforms(joint_positions, out_header.stamp);
            auto fixed_transforms = _joint_state_to_tf.getFixedTransforms();
            fixed_transforms.push_back(_world_to_base_link_tf);

            // // This is temporary because you know...Do not have time to read all from URDF, because who cares...
            // geometry_msgs::msg::TransformStamped temp_tf;
            // temp_tf.header.frame_id = _robot_info.robot_prefix + "_gripper";
            // temp_tf.child_frame_id = _robot_info.robot_prefix + "_gripper_left_finger";
            // moving_transforms.push_back(temp_tf);

            // temp_tf.header.frame_id = _robot_info.robot_prefix + "_gripper";
            // temp_tf.child_frame_id = _robot_info.robot_prefix + "_gripper_right_finger";
            // moving_transforms.push_back(temp_tf);
            // // ...and here this temporary stuff ends

            std::for_each(moving_transforms.begin(), moving_transforms.end(), update_frame_ids);
            std::for_each(fixed_transforms.begin(), fixed_transforms.end(), update_frame_ids);

            std::this_thread::sleep_until(start_move_time + time_from_start);

            _tf_broadcaster->sendTransform(moving_transforms);
            _static_tf_broadcaster->sendTransform(fixed_transforms);
        }
    }

    std::optional<geometry_msgs::msg::TransformStamped> MovementVisualization::_getWorldToBaseLinkTf()
    {
        RCLCPP_DEBUG(get_logger(), "Get transforms for fixed joints");
        std::optional<geometry_msgs::msg::TransformStamped> tf_transform;

        // Transform from "world" to *_link_0
        std::string link_0_name = _robot_info.robot_prefix + "_link_0";
        if (auto link_0_to_world = helpers::vision::getTransformStamped(_world_name, link_0_name, std::chrono::seconds(5)))
        {
            RCLCPP_INFO_STREAM(get_logger(), "Received transform from \"" << link_0_name << "\" to \"" << _world_name << "\"");
            tf_transform = link_0_to_world;
        }
        else
            return std::nullopt;
        return tf_transform;
    }

    geometry_msgs::msg::TransformStamped MovementVisualization::_getEndEffectorTransform(
        const std::vector<geometry_msgs::msg::TransformStamped> &fixed_transforms,
        const std::vector<geometry_msgs::msg::TransformStamped> &moving_transforms)
    {
        geometry_msgs::msg::TransformStamped end_effector_transform;

        tf2_ros::Buffer buffer(get_clock());
        auto set_transforms = [this](tf2_ros::Buffer &buffer, const std::vector<geometry_msgs::msg::TransformStamped> &transforms, const bool is_static)
        {
            const std::string authority = "Authority undetectable";
            for (auto &transform : transforms)
            {
                try
                {
                    buffer.setTransform(transform, authority, is_static);
                }
                catch (const tf2::TransformException &ex)
                {
                    std::string temp = ex.what();
                    RCLCPP_ERROR(
                        get_logger(),
                        "Failure to set received transform from %s to %s with error: %s\n",
                        transform.child_frame_id.c_str(),
                        transform.header.frame_id.c_str(), temp.c_str());
                }
            }
        };
        set_transforms(buffer, fixed_transforms, true);
        set_transforms(buffer, moving_transforms, false);

        try
        {
            end_effector_transform = buffer.lookupTransform(_world_name, _end_effector_name, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            std::cout << "Exception thrown:" << ex.what() << std::endl;
            std::cout << "The current list of frames is:" << std::endl;
            std::cout << buffer.allFramesAsString() << std::endl;
        }
        return end_effector_transform;
    }

} // namespace visualization_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(visualization_tools::MovementVisualization)
