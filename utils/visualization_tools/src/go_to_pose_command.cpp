#include "visualization_tools/go_to_pose_command.hpp"

namespace visualization_tools
{
    GoToPoseCommand::GoToPoseCommand(const rclcpp::NodeOptions &options)
        : Node("go_to_pose_command", options)
    //  _lock_marker_interaction(false)
    {
        helpers::commons::setLoggerLevelFromParameter(this);
        RCLCPP_INFO(get_logger(), "Initialize go to pose command interactive marker server");
        _interactive_markers_server = std::make_shared<interactive_markers::InteractiveMarkerServer>("go_to_pose_commands", this);

        _setupMenuEntries(_menu_handler);
        _createGoToPoseCommandMarker();

        // Read robot info
        if (auto robot_info = helpers::commons::getRobotInfo())
            _robot_info = *robot_info;
        else
        {
            RCLCPP_FATAL(get_logger(), "Cannot read robot info. Exiting...");
            std::exit(1);
        }

        _movement_action_clients[GENERATE_PATH_NAME] = rclcpp_action::create_client<GeneratePathPose>(this, GENERATE_PATH_NAME);
        // _generate_path_pick_client = rclcpp_action::create_client<GeneratePathPick>(this, GENERATE_PATH_PICK_NAME);
        _movement_action_clients[GENERATE_TRAJECTORY_NAME] = rclcpp_action::create_client<GenerateTrajectory>(this, GENERATE_TRAJECTORY_NAME);
        _movement_action_clients[EXECUTE_MOVE_NAME] = rclcpp_action::create_client<GenerateTrajectory>(this, EXECUTE_MOVE_NAME);
    }

    void GoToPoseCommand::_markerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
            _request_go_to_pose = feedback->pose;
    }

    void GoToPoseCommand::_movementFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        // if (_lock_marker_interaction)
        // {
        //     RCLCPP_WARN(get_logger(), "Waiting for action server to finish processing");
        //     return;
        // }
        if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT)
        {
            if (_menu_entries[feedback->menu_entry_id] == GENERATE_PATH_NAME)
            {
                _sendGeneratePathGoal(_request_go_to_pose);
            }
            // else if (_menu_entries[feedback->menu_entry_id] == GENERATE_PATH_PICK_NAME)
            // {
            //     _sendGeneratePathPickGoal(_request_go_to_pose);
            // }
            else if (_menu_entries[feedback->menu_entry_id] == GENERATE_TRAJECTORY_NAME)
            {
                _sendGenerateTrajectoryGoal();
            }
            else if (_menu_entries[feedback->menu_entry_id] == EXECUTE_MOVE_NAME)
            {
                _sendExecuteMoveGoal();
            }
            else
                RCLCPP_WARN(get_logger(), "Invalid menu entry");
        }
    }

    void GoToPoseCommand::_resetMarkerPoseFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        RCLCPP_DEBUG(get_logger(), "Reset marker pose feedback");
        if (auto ee_pose = _getEndEffectorPose())
        {
            _interactive_markers_server->setPose(feedback->marker_name, *ee_pose);
            _request_go_to_pose = *ee_pose;
        }
        else
            RCLCPP_WARN(get_logger(), "Cannot read end effector pose");

        _interactive_markers_server->applyChanges();
    }

    std::optional<geometry_msgs::msg::Pose> GoToPoseCommand::_getEndEffectorPose()
    {
        if (auto ee_effector_pose_opt = helpers::vision::getTransformStamped("world", _robot_info.connection))
        {
            geometry_msgs::msg::Pose ee_pose;
            ee_pose.position.x = ee_effector_pose_opt->transform.translation.x;
            ee_pose.position.y = ee_effector_pose_opt->transform.translation.y;
            ee_pose.position.z = ee_effector_pose_opt->transform.translation.z;

            ee_pose.orientation.x = ee_effector_pose_opt->transform.rotation.x;
            ee_pose.orientation.y = ee_effector_pose_opt->transform.rotation.y;
            ee_pose.orientation.z = ee_effector_pose_opt->transform.rotation.z;
            ee_pose.orientation.w = ee_effector_pose_opt->transform.rotation.w;
            return ee_pose;
        }
        return std::nullopt;
    }

    void GoToPoseCommand::_createGoToPoseCommandMarker()
    {
        visualization_msgs::msg::InteractiveMarker int_marker;
        int_marker.header.frame_id = "world";
        int_marker.pose.position.x = 0.5;
        int_marker.pose.position.y = 0.0;
        int_marker.pose.position.z = 0.1;
        int_marker.scale = 0.2;
        int_marker.name = "go_to_pose";

        auto add_axis_control_marker = [this](visualization_msgs::msg::InteractiveMarker &interactive_marker, const std::string &axis)
        {
            visualization_msgs::msg::InteractiveMarkerControl control;
            Eigen::Quaternionf orien;
            if (axis == "x")
                orien = Eigen::Quaternionf(1.0, 1.0, 0.0, 0.0);
            else if (axis == "y")
                orien = Eigen::Quaternionf(1.0, 0.0, 1.0, 0.0);
            else if (axis == "z")
                orien = Eigen::Quaternionf(1.0, 0.0, 0.0, 1.0);
            else
                return;
            orien.normalize();
            control.orientation = _eigenToRos(orien);
            control.name = "rotate_" + axis;
            control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
            interactive_marker.controls.push_back(control);
            control.name = "move_" + axis;
            control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
            interactive_marker.controls.push_back(control);
        };

        add_axis_control_marker(int_marker, "x");
        add_axis_control_marker(int_marker, "y");
        add_axis_control_marker(int_marker, "z");

        // Menu select what to do
        visualization_msgs::msg::InteractiveMarkerControl control;
        control.name = "move_rotate_3D";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
        visualization_msgs::msg::Marker marker = _makeSphere(int_marker);
        marker.color.r = 1.0;
        marker.color.g = 0.55;
        marker.color.b = 0.0;
        control.markers.push_back(marker);
        control.always_visible = true;
        int_marker.controls.push_back(control);

        _interactive_markers_server->insert(int_marker);
        _interactive_markers_server->setCallback(int_marker.name, std::bind(&GoToPoseCommand::_markerFeedback, this, std::placeholders::_1));
        _menu_handler.apply(*_interactive_markers_server, int_marker.name);
        _interactive_markers_server->applyChanges();
    }

    void GoToPoseCommand::_sendGeneratePathGoal(const geometry_msgs::msg::Pose &requested_end_effector_pose)
    {
        // _lock_marker_interaction = true;
        RCLCPP_INFO(get_logger(), "Waiting for generate path pose action server");
        auto action_client = dynamic_cast<rclcpp_action::Client<GeneratePathPose> *>(_movement_action_clients[GENERATE_PATH_NAME].get());
        if (!action_client->wait_for_action_server(WAITING_FOR_ACTION_TIMEOUT))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            // _lock_marker_interaction = false;
            return;
        }

        auto goal_msg = GeneratePathPose::Goal();
        goal_msg.end_effector_pose = requested_end_effector_pose;

        auto send_goal_options = rclcpp_action::Client<GeneratePathPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](std::shared_future<GoalHandleGeneratePathPose::SharedPtr>) {};
        send_goal_options.feedback_callback = [this](GoalHandleGeneratePathPose::SharedPtr, const std::shared_ptr<const GeneratePathPose::Feedback>) {};
        send_goal_options.result_callback =
            [this](const GoalHandleGeneratePathPose::WrappedResult &result)
        {
            _resultCallback(result.code, "Generate path");
        };
        RCLCPP_INFO(get_logger(), "Sending request to generate path");
        action_client->async_send_goal(goal_msg, send_goal_options);
    }

    // void GoToPoseCommand::_sendGeneratePathPickGoal(const geometry_msgs::msg::Pose &requested_end_effector_pose)
    // {
    //     _lock_marker_interaction = true;
    //     RCLCPP_INFO(get_logger(), "Waiting for generate path pick action server");
    //     if (!_generate_path_pick_client->wait_for_action_server(WAITING_FOR_ACTION_TIMEOUT))
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    //         _lock_marker_interaction = false;
    //         return;
    //     }

    //     auto goal_msg = GeneratePathPick::Goal();
    //     // Values taken from test scripts in ~/ros2_ws/src/avena_ros2/system_tests/scripts/action_prepare_pick_and_place_data.sh
    //     goal_msg.constrain_value = 40;
    //     goal_msg.ik_trials_number = 150;
    //     goal_msg.max_final_states = 4;
    //     goal_msg.ompl_compare_trials = 1;
    //     goal_msg.min_path_points = 200;
    //     goal_msg.max_time = 15;
    //     goal_msg.max_simplification_time = 15;

    //     goal_msg.grasp_pose.grasp_pose = requested_end_effector_pose;

    //     // Compute pregrasp pose
    //     const float pregrasp_distance = 0.1;
    //     Eigen::Affine3f grasp;
    //     helpers::converters::geometryToEigenAffine(requested_end_effector_pose, grasp);
    //     Eigen::Vector3f pregrasp_position = grasp.translation() - (grasp.rotation().col(0) * pregrasp_distance);
    //     Eigen::Affine3f pregrasp_pose = Eigen::Translation3f(pregrasp_position) * Eigen::Quaternionf(grasp.rotation());
    //     helpers::converters::eigenAffineToGeometry(pregrasp_pose, goal_msg.grasp_pose.pregrasp_pose);

    //     auto send_goal_options = rclcpp_action::Client<GeneratePathPick>::SendGoalOptions();
    //     send_goal_options.goal_response_callback = std::bind(&GoToPoseCommand::_goalResponseGeneratePathPickCallback, this, std::placeholders::_1);
    //     send_goal_options.feedback_callback = std::bind(&GoToPoseCommand::_feedbackGeneratePathPickCallback, this, std::placeholders::_1, std::placeholders::_2);
    //     send_goal_options.result_callback = std::bind(&GoToPoseCommand::_resultGeneratePathPickCallback, this, std::placeholders::_1);
    //     RCLCPP_INFO(get_logger(), "Sending request to generate path pick");
    //     _generate_path_pick_client->async_send_goal(goal_msg, send_goal_options);
    // }

    void GoToPoseCommand::_sendGenerateTrajectoryGoal()
    {
        // _lock_marker_interaction = true;
        RCLCPP_INFO(get_logger(), "Waiting for generate trajectory action server");
        auto action_client = dynamic_cast<rclcpp_action::Client<GenerateTrajectory> *>(_movement_action_clients[GENERATE_TRAJECTORY_NAME].get());
        if (!action_client->wait_for_action_server(WAITING_FOR_ACTION_TIMEOUT))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            // _lock_marker_interaction = false;
            return;
        }
        auto goal_msg = GenerateTrajectory::Goal();
        auto send_goal_options = rclcpp_action::Client<GenerateTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](std::shared_future<GoalHandleGenerateTrajectory::SharedPtr>) {};
        send_goal_options.feedback_callback = [this](GoalHandleGenerateTrajectory::SharedPtr, const std::shared_ptr<const GenerateTrajectory::Feedback>) {};
        send_goal_options.result_callback =
            [this](const GoalHandleGenerateTrajectory::WrappedResult &result)
        {
            _resultCallback(result.code, "Generate trajectory");
        };
        RCLCPP_INFO(get_logger(), "Sending request to generate trajectory");
        action_client->async_send_goal(goal_msg, send_goal_options);
    }

    void GoToPoseCommand::_sendExecuteMoveGoal()
    {
        // _lock_marker_interaction = true;
        RCLCPP_INFO(get_logger(), "Waiting for execute move action server");
        auto action_client = dynamic_cast<rclcpp_action::Client<ExecuteMove> *>(_movement_action_clients[EXECUTE_MOVE_NAME].get());
        if (!action_client->wait_for_action_server(WAITING_FOR_ACTION_TIMEOUT))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            // _lock_marker_interaction = false;
            return;
        }
        auto goal_msg = ExecuteMove::Goal();
        auto send_goal_options = rclcpp_action::Client<ExecuteMove>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](std::shared_future<GoalHandleExecuteMove::SharedPtr>) {};
        send_goal_options.feedback_callback = [this](GoalHandleExecuteMove::SharedPtr, const std::shared_ptr<const ExecuteMove::Feedback>) {};
        send_goal_options.result_callback =
            [this](const GoalHandleExecuteMove::WrappedResult &result)
        {
            _resultCallback(result.code, "Execute move");
        };
        RCLCPP_INFO(get_logger(), "Sending request to execute move");
        action_client->async_send_goal(goal_msg, send_goal_options);
    }

    void GoToPoseCommand::_resultCallback(const rclcpp_action::ResultCode &result_code, const std::string &message)
    {
        // _lock_marker_interaction = false;
        switch (result_code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), message + ": success");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), message + ": goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), message + ": goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), message + ": unknown result code");
            break;
        }
    }

    void GoToPoseCommand::_setupMenuEntries(interactive_markers::MenuHandler &menu_handler)
    {
        // Generate path to marker pose
        auto handle = menu_handler.insert("Generate path", std::bind(&GoToPoseCommand::_movementFeedback, this, std::placeholders::_1));
        _menu_entries[handle] = GENERATE_PATH_NAME;

        // handle = menu_handler.insert("Generate path pick", std::bind(&GoToPoseCommand::_movementFeedback, this, std::placeholders::_1));
        // _menu_entries[handle] = GENERATE_PATH_PICK_NAME;
        // handle = menu_handler.insert("Generate path place", std::bind(&GoToPoseCommand::_movementFeedback, this, std::placeholders::_1));
        // _menu_entries[handle] = GENERATE_PATH_PLACE_NAME;

        // Generate trajectory action
        handle = menu_handler.insert("Generate trajectory", std::bind(&GoToPoseCommand::_movementFeedback, this, std::placeholders::_1));
        _menu_entries[handle] = GENERATE_TRAJECTORY_NAME;

        // Execute arm movement action
        handle = menu_handler.insert("Execute move", std::bind(&GoToPoseCommand::_movementFeedback, this, std::placeholders::_1));
        _menu_entries[handle] = EXECUTE_MOVE_NAME;

        // Reset marker pose to current end effector pose
        handle = menu_handler.insert("Reset marker", std::bind(&GoToPoseCommand::_resetMarkerPoseFeedback, this, std::placeholders::_1));
    }

    visualization_msgs::msg::Marker GoToPoseCommand::_makeBox(const visualization_msgs::msg::InteractiveMarker &msg)
    {
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.scale.x = msg.scale * 0.4;
        marker.scale.y = msg.scale * 0.4;
        marker.scale.z = msg.scale * 0.4;
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
        marker.color.a = 1.0;
        return marker;
    }

    visualization_msgs::msg::Marker GoToPoseCommand::_makeSphere(const visualization_msgs::msg::InteractiveMarker &msg)
    {
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = msg.scale * 0.6;
        marker.scale.y = msg.scale * 0.6;
        marker.scale.z = msg.scale * 0.6;
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
        marker.color.a = 0.8;
        return marker;
    }

    geometry_msgs::msg::Quaternion GoToPoseCommand::_eigenToRos(const Eigen::Quaternionf &orien)
    {
        geometry_msgs::msg::Quaternion ros_orien;
        ros_orien.x = orien.x();
        ros_orien.y = orien.y();
        ros_orien.z = orien.z();
        ros_orien.w = orien.w();
        return ros_orien;
    }

} // namespace visualization_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(visualization_tools::GoToPoseCommand)
