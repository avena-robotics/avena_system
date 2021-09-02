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

        // _movement_action_clients[GENERATE_PATH_NAME] = rclcpp_action::create_client<GeneratePathPose>(this, GENERATE_PATH_NAME);
        // _generate_path_pick_client = rclcpp_action::create_client<GeneratePathPick>(this, GENERATE_PATH_PICK_NAME);
        _movement_action_clients[GENERATE_TRAJECTORY_NAME] = rclcpp_action::create_client<GenerateTrajectory>(this, GENERATE_TRAJECTORY_NAME);
        // _movement_action_clients[EXECUTE_MOVE_NAME] = rclcpp_action::create_client<GenerateTrajectory>(this, EXECUTE_MOVE_NAME);

        _movement_sequence_insert_client = create_client<MovementSequenceInsert>("movement_sequence_insert");
        _octomap_insert_client = create_client<OctomapInsert>("scene_insert");
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
            // if (_menu_entries[feedback->menu_entry_id] == GENERATE_PATH_NAME)
            // {
            //     _sendGeneratePathGoal(_request_go_to_pose);
            // }
            // else if (_menu_entries[feedback->menu_entry_id] == GENERATE_PATH_PICK_NAME)
            // {
            //     _sendGeneratePathPickGoal(_request_go_to_pose);
            // }
            if (_menu_entries[feedback->menu_entry_id] == GENERATE_TRAJECTORY_NAME)
            {
                std::thread([this]()
                            {
                                try
                                {
                                    _writeMovementSequence();
                                    _writeSceneOctomap();
                                }
                                catch (const std::exception &e)
                                {
                                    RCLCPP_ERROR(get_logger(), "Error occured while inserting movement sequence to server. Error: %s", e.what());
                                    return;
                                }
                                try
                                {
                                    _sendGenerateTrajectoryGoal();
                                }
                                catch (const std::exception &e)
                                {
                                    RCLCPP_ERROR(get_logger(), "Error occured while sending generate trajectory request. Error: %s", e.what());
                                    return;
                                }
                            })
                    .detach();
            }
            // else if (_menu_entries[feedback->menu_entry_id] == EXECUTE_MOVE_NAME)
            // {
            //     _sendExecuteMoveGoal();
            // }
            else
                RCLCPP_WARN(get_logger(), "Invalid menu entry");
        }
    }

    void GoToPoseCommand::_sendGenerateTrajectoryGoal()
    {
        // _lock_marker_interaction = true;
        RCLCPP_INFO(get_logger(), "Waiting for generate trajectory action server");
        auto action_client = dynamic_cast<rclcpp_action::Client<GenerateTrajectory> *>(_movement_action_clients[GENERATE_TRAJECTORY_NAME].get());
        if (!action_client->wait_for_action_server(WAITING_FOR_ACTION_TIMEOUT))
            throw std::runtime_error("Action server not available after waiting");

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

    void GoToPoseCommand::_writeMovementSequence()
    {
        auto movement_sequence_req = std::make_shared<MovementSequenceInsert::Request>();
        if (!_movement_sequence_insert_client->wait_for_service(std::chrono::seconds(3)))
            throw std::runtime_error("Cannot connect to server to insert movement sequence");

        EndEffectorPose ee_pose;
        ee_pose.pose = _request_go_to_pose;
        ee_pose.path_type = EndEffectorPose::LINEAR;
        movement_sequence_req->data.push_back(ee_pose);

        RCLCPP_INFO(get_logger(), "Trying to send movement sequence to server");
        auto movement_sequence_res = _movement_sequence_insert_client->async_send_request(movement_sequence_req);

        // auto future_return_code = rclcpp::spin_until_future_complete(shared_from_this(), movement_sequence_res, std::chrono::seconds(10));
        // if (future_return_code == rclcpp::FutureReturnCode::INTERRUPTED)
        //     throw std::runtime_error("INTERRUPTED");

        // if (future_return_code == rclcpp::FutureReturnCode::TIMEOUT)
        //     throw std::runtime_error("TIMEOUT");

        if (movement_sequence_res.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
            throw std::runtime_error("Server not responding when trying to insert movement sequence");

        if (!movement_sequence_res.get()->result.data)
            throw std::runtime_error("Server not able to save movement sequence data");
    }

    void GoToPoseCommand::_writeSceneOctomap()
    {
        auto octomap_req = std::make_shared<OctomapInsert::Request>();
        if (!_octomap_insert_client->wait_for_service(std::chrono::seconds(3)))
            throw std::runtime_error("Cannot connect to server to insert movement sequence");

        pcl::PointCloud<pcl::PointXYZ>::Ptr octomap = helpers::vision::makeSharedPcl<pcl::PointXYZ>();
        helpers::converters::pclToRosPtcld<pcl::PointXYZ>(octomap, octomap_req->data.octomap.scene_octomap);

        RCLCPP_INFO(get_logger(), "Trying to send octomap to server");
        auto octomap_res = _octomap_insert_client->async_send_request(octomap_req);

        if (octomap_res.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
            throw std::runtime_error("Server not responding when trying to insert octomap");

        if (!octomap_res.get()->result.data)
            throw std::runtime_error("Server not able to save octomap data");
    }

    // void GoToPoseCommand::_sendExecuteMoveGoal()
    // {
    //     // _lock_marker_interaction = true;
    //     RCLCPP_INFO(get_logger(), "Waiting for execute move action server");
    //     auto action_client = dynamic_cast<rclcpp_action::Client<ExecuteMove> *>(_movement_action_clients[EXECUTE_MOVE_NAME].get());
    //     if (!action_client->wait_for_action_server(WAITING_FOR_ACTION_TIMEOUT))
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    //         // _lock_marker_interaction = false;
    //         return;
    //     }
    //     auto goal_msg = ExecuteMove::Goal();
    //     auto send_goal_options = rclcpp_action::Client<ExecuteMove>::SendGoalOptions();
    //     send_goal_options.goal_response_callback = [this](std::shared_future<GoalHandleExecuteMove::SharedPtr>) {};
    //     send_goal_options.feedback_callback = [this](GoalHandleExecuteMove::SharedPtr, const std::shared_ptr<const ExecuteMove::Feedback>) {};
    //     send_goal_options.result_callback =
    //         [this](const GoalHandleExecuteMove::WrappedResult &result)
    //     {
    //         _resultCallback(result.code, "Execute move");
    //     };
    //     RCLCPP_INFO(get_logger(), "Sending request to execute move");
    //     action_client->async_send_goal(goal_msg, send_goal_options);
    // }

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

    void GoToPoseCommand::_setupMenuEntries(interactive_markers::MenuHandler &menu_handler)
    {
        interactive_markers::MenuHandler::EntryHandle handle;

        // // Generate path to marker pose
        // auto handle = menu_handler.insert("Generate path", std::bind(&GoToPoseCommand::_movementFeedback, this, std::placeholders::_1));
        // _menu_entries[handle] = GENERATE_PATH_NAME;

        // handle = menu_handler.insert("Generate path pick", std::bind(&GoToPoseCommand::_movementFeedback, this, std::placeholders::_1));
        // _menu_entries[handle] = GENERATE_PATH_PICK_NAME;
        // handle = menu_handler.insert("Generate path place", std::bind(&GoToPoseCommand::_movementFeedback, this, std::placeholders::_1));
        // _menu_entries[handle] = GENERATE_PATH_PLACE_NAME;

        // Generate trajectory action
        handle = menu_handler.insert("Generate trajectory", std::bind(&GoToPoseCommand::_movementFeedback, this, std::placeholders::_1));
        _menu_entries[handle] = GENERATE_TRAJECTORY_NAME;

        // // Execute arm movement action
        // handle = menu_handler.insert("Execute move", std::bind(&GoToPoseCommand::_movementFeedback, this, std::placeholders::_1));
        // _menu_entries[handle] = EXECUTE_MOVE_NAME;

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
