#include "visualization_tools/go_to_pose_command.hpp"

namespace visualization_tools
{
    GoToPoseCommand::GoToPoseCommand(const rclcpp::NodeOptions &options)
        : Node("go_to_pose_command", options),
          _marker_count(0)
    //  _lock_marker_interaction(false)
    {
        helpers::commons::setLoggerLevelFromParameter(this);
        RCLCPP_INFO(get_logger(), "Initialize go to pose command interactive marker server");
        _interactive_markers_server = std::make_shared<interactive_markers::InteractiveMarkerServer>("/go_to_pose_commands", this);

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

        _movement_action_clients[GENERATE_TRAJECTORY_NAME] = rclcpp_action::create_client<GenerateTrajectory>(this, GENERATE_TRAJECTORY_NAME);
        _movement_sequence_insert_client = create_client<MovementSequenceInsert>("movement_sequence_insert");
        _octomap_insert_client = create_client<OctomapInsert>("scene_insert");
        _motion_planning_info_markers_pub = create_publisher<visualization_msgs::msg::MarkerArray>("/motion_planning_info", 1);
        _sequence_poses_pub = create_publisher<geometry_msgs::msg::PoseArray>("/sequence_poses", 1);
    }

    void GoToPoseCommand::_markerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
        {
            _request_go_to_pose = feedback->pose;
        }
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
            const MenuEntries_e &menu_entry = _menu_entries[feedback->menu_entry_id];
            if (menu_entry == MenuEntries_e::PATH || menu_entry == MenuEntries_e::LINEAR_PATH || menu_entry == MenuEntries_e::ORIENTATION_PATH || menu_entry == MenuEntries_e::LO_PATH)
            {
                _saveSequenceSegmentToBuffer(menu_entry);
            }
            else if (menu_entry == MenuEntries_e::CLEAR_SEQUENCE)
            {
                _sequence_to_execute.clear();
            }
            else if (menu_entry == MenuEntries_e::START_PLANNING)
            {
                std::thread([=]()
                            {
                                try
                                {
                                    _deleteMotionPlanningInfoMarkers();
                                    _writeMovementSequenceToServer();
                                    // _writeSceneOctomapToServer();
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

    void GoToPoseCommand::_deleteMotionPlanningInfoMarkers()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = now();
        marker.ns = "/";
        marker.action = visualization_msgs::msg::Marker::DELETEALL;

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        _motion_planning_info_markers_pub->publish(marker_array);
        _marker_count = 0;

        // Clear path
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.frame_id = "world";
        pose_array.header.stamp = now();
        pose_array.poses.clear();
        _sequence_poses_pub->publish(pose_array);
    }

    visualization_msgs::msg::Marker GoToPoseCommand::_getConstraintLine(const geometry_msgs::msg::Pose &start_end_effector_pose, const geometry_msgs::msg::Pose &requested_end_effector_pose)
    {
        Eigen::Affine3d requested_ee_pose;
        tf2::fromMsg(requested_end_effector_pose, requested_ee_pose);

        Eigen::Affine3d current_ee_pose;
        tf2::fromMsg(start_end_effector_pose, current_ee_pose);

        double dist = (requested_ee_pose.translation() - current_ee_pose.translation()).norm();
        std::vector<double> dims = {0.001, 0.001, dist};

        geometry_msgs::msg::Pose constraint_area_pose;
        Eigen::Vector3d middle_point = (requested_ee_pose.translation() + current_ee_pose.translation()) / 2.0;
        constraint_area_pose.position = tf2::toMsg(middle_point);

        Eigen::Vector3d direction_axis = (requested_ee_pose.translation() - current_ee_pose.translation()).normalized();
        Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), direction_axis);
        constraint_area_pose.orientation = tf2::toMsg(rotation);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = now();
        marker.ns = "/";
        marker.id = _marker_count++;

        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(0);

        marker.color.a = 0.5;
        marker.pose = constraint_area_pose;
        marker.scale.x = dims.at(0);
        marker.scale.y = dims.at(1);
        marker.scale.z = dims.at(2);

        return marker;
    }

    void GoToPoseCommand::_saveSequenceSegmentToBuffer(const MenuEntries_e &path_type)
    {
        EndEffectorPose ee_pose;
        ee_pose.pose = _request_go_to_pose;

        if (path_type == MenuEntries_e::LINEAR_PATH)
        {
            // _drawConstraintLine(*_getEndEffectorPose(), _request_go_to_pose);
            ee_pose.path_type = EndEffectorPose::LINEAR;
        }
        else if (path_type == MenuEntries_e::ORIENTATION_PATH)
        {
            ee_pose.path_type = EndEffectorPose::ORIENTATION;
        }
        else if (path_type == MenuEntries_e::LO_PATH)
        {
            ee_pose.path_type = EndEffectorPose::LO;
        }
        else if (path_type == MenuEntries_e::PATH)
        {
            ee_pose.path_type = EndEffectorPose::PATH;
        }

        else
        {
            throw std::runtime_error("Invalid path type");
        }
        _sequence_to_execute.push_back(ee_pose);
    }

    void GoToPoseCommand::_writeMovementSequenceToServer()
    {
        if (_sequence_to_execute.size() == 0)
            throw std::runtime_error("There are no end effector sequence to execute in the buffer");

        auto movement_sequence_req = std::make_shared<MovementSequenceInsert::Request>();
        if (!_movement_sequence_insert_client->wait_for_service(std::chrono::seconds(3)))
            throw std::runtime_error("Cannot connect to server to insert movement sequence");
        movement_sequence_req->data = _sequence_to_execute;
        RCLCPP_INFO(get_logger(), "Trying to send movement sequence to server. Waiting 10 seconds");
        auto movement_sequence_res = _movement_sequence_insert_client->async_send_request(movement_sequence_req);
        if (movement_sequence_res.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
            throw std::runtime_error("Server not responding when trying to insert movement sequence");

        if (!movement_sequence_res.get()->result.data)
            throw std::runtime_error("Server not able to save movement sequence data");

        // Draw linear constraints if there are some in Rviz
        geometry_msgs::msg::Pose start_ee_pose = *_getEndEffectorPose();
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.frame_id = "world";
        pose_array.header.stamp = now();

        visualization_msgs::msg::MarkerArray marker_array;
        for (size_t i = 0; i < _sequence_to_execute.size(); i++)
        {
            const EndEffectorPose &ee_requested_pose = _sequence_to_execute[i];
            if (ee_requested_pose.path_type == EndEffectorPose::LINEAR)
            {
                marker_array.markers.push_back(_getConstraintLine(start_ee_pose, ee_requested_pose.pose));
            }
            pose_array.poses.push_back(ee_requested_pose.pose);

            start_ee_pose = ee_requested_pose.pose;
        }
        _sequence_poses_pub->publish(pose_array);
        _motion_planning_info_markers_pub->publish(marker_array);
        _sequence_to_execute.clear();
    }

    // void GoToPoseCommand::_writeSceneOctomapToServer()
    // {
    //     auto octomap_req = std::make_shared<OctomapInsert::Request>();
    //     if (!_octomap_insert_client->wait_for_service(std::chrono::seconds(3)))
    //         throw std::runtime_error("Cannot connect to server to insert movement sequence");

    //     pcl::PointCloud<pcl::PointXYZ>::Ptr octomap = helpers::vision::makeSharedPcl<pcl::PointXYZ>();
    //     helpers::converters::pclToRosPtcld<pcl::PointXYZ>(octomap, octomap_req->data.octomap.scene_octomap);

    //     RCLCPP_INFO(get_logger(), "Trying to send octomap to server");
    //     auto octomap_res = _octomap_insert_client->async_send_request(octomap_req);

    //     if (octomap_res.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    //         throw std::runtime_error("Server not responding when trying to insert octomap");

    //     if (!octomap_res.get()->result.data)
    //         throw std::runtime_error("Server not able to save octomap data");
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

        // Generate trajectory action (path)
        handle = menu_handler.insert("Add waypoint (PATH)", std::bind(&GoToPoseCommand::_movementFeedback, this, std::placeholders::_1));
        _menu_entries[handle] = MenuEntries_e::PATH;

        // Generate trajectory action (linear path)
        handle = menu_handler.insert("Add waypoint (LINEAR)", std::bind(&GoToPoseCommand::_movementFeedback, this, std::placeholders::_1));
        _menu_entries[handle] = MenuEntries_e::LINEAR_PATH;

        // Generate trajectory action (orientation path)
        handle = menu_handler.insert("Add waypoint (ORIENTATION)", std::bind(&GoToPoseCommand::_movementFeedback, this, std::placeholders::_1));
        _menu_entries[handle] = MenuEntries_e::ORIENTATION_PATH;

        // Generate trajectory action (ol path)
        handle = menu_handler.insert("Add waypoint (L+O)", std::bind(&GoToPoseCommand::_movementFeedback, this, std::placeholders::_1));
        _menu_entries[handle] = MenuEntries_e::LO_PATH;

        // Clear buffer
        handle = menu_handler.insert("Clear sequence", std::bind(&GoToPoseCommand::_movementFeedback, this, std::placeholders::_1));
        _menu_entries[handle] = MenuEntries_e::CLEAR_SEQUENCE;

        // Trigger motion planning
        handle = menu_handler.insert("Start motion planning", std::bind(&GoToPoseCommand::_movementFeedback, this, std::placeholders::_1));
        _menu_entries[handle] = MenuEntries_e::START_PLANNING;

        // Dummy entry to separate motion planning from utils options
        handle = menu_handler.insert("---", [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & /*feedback*/) {});

        // Reset marker pose to current end effector pose
        handle = menu_handler.insert("Reset marker position", std::bind(&GoToPoseCommand::_resetMarkerPoseFeedback, this, std::placeholders::_1));

        // Remove all constraints markers
        handle = menu_handler.insert("Delete constraints markers",
                                     [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & /*feedback*/)
                                     {
                                         _deleteMotionPlanningInfoMarkers();
                                     });
    }

    // void GoToPoseCommand::_drawConstraintArea(const geometry_msgs::msg::Pose &start_end_effector_pose, const geometry_msgs::msg::Pose &requested_end_effector_pose)
    // {
    //     std::vector<double> dims = {std::abs(requested_end_effector_pose.position.x - start_end_effector_pose.position.x),
    //                                 std::abs(requested_end_effector_pose.position.y - start_end_effector_pose.position.y),
    //                                 std::abs(requested_end_effector_pose.position.z - start_end_effector_pose.position.z)};

    //     geometry_msgs::msg::Pose pose;
    //     pose.position.x = (requested_end_effector_pose.position.x + start_end_effector_pose.position.x) / 2.0;
    //     pose.position.y = (requested_end_effector_pose.position.y + start_end_effector_pose.position.y) / 2.0;
    //     pose.position.z = (requested_end_effector_pose.position.z + start_end_effector_pose.position.z) / 2.0;
    //     pose.orientation.x = 0;
    //     pose.orientation.y = 0;
    //     pose.orientation.z = 0;
    //     pose.orientation.w = 1;

    //     visualization_msgs::msg::Marker marker;
    //     marker.header.frame_id = "world";
    //     marker.header.stamp = now();
    //     marker.ns = "/";
    //     marker.id = _marker_count++;

    //     marker.type = visualization_msgs::msg::Marker::CUBE;
    //     marker.action = visualization_msgs::msg::Marker::ADD;
    //     marker.lifetime = rclcpp::Duration::from_seconds(0);

    //     marker.color.a = 0.5;
    //     marker.pose = pose;
    //     marker.scale.x = dims.at(0);
    //     marker.scale.y = dims.at(1);
    //     marker.scale.z = dims.at(2);

    //     _motion_planning_info_markers_pub->publish(marker);
    // }

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
