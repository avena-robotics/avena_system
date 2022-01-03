#include "motion_planning/motion_planning.hpp"

namespace motion_planning
{
    MotionPlanning::MotionPlanning(const rclcpp::NodeOptions &options)
        : Node("motion_planning", options)
    {
        helpers::commons::setLoggerLevelFromParameter(this);
        status = custom_interfaces::msg::Heartbeat::STOPPED;
        _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
    }

    MotionPlanning::~MotionPlanning()
    {
        shutDownNode();
    }

    void MotionPlanning::initNode()
    {
        RCLCPP_DEBUG(get_logger(), "Initializing node");
        status = custom_interfaces::msg::Heartbeat::STARTING;
        try
        {
            _initialize();
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN_STREAM(get_logger(), "Error occured while initializing motion planning node. Error: " << e.what());
            status = custom_interfaces::msg::Heartbeat::STOPPED;
            return;
        }
        status = custom_interfaces::msg::Heartbeat::RUNNING;
    }

    void MotionPlanning::shutDownNode()
    {
        RCLCPP_DEBUG(get_logger(), "Shutting down node");
        status = custom_interfaces::msg::Heartbeat::STOPPING;
        _shutdown();
        status = custom_interfaces::msg::Heartbeat::STOPPED;
    }

    void MotionPlanning::_initialize()
    {
        _action_server_motion_planning = rclcpp_action::create_server<MotionPlanningAction>(
            this, "generate_trajectory",
            std::bind(&MotionPlanning::_handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionPlanning::_handleCancel, this, std::placeholders::_1),
            std::bind(&MotionPlanning::_handleAccepted, this, std::placeholders::_1));

        _trajectory_insert_client = create_client<TrajectoryInsert>("trajectory_insert");
        _octomap_select_client = create_client<OctomapSelect>("scene_select");
        _movement_sequence_select_client = create_client<MovementSequenceSelect>("movement_sequence_select");

        // Check whether data store servers are available
        const auto timeout = 1s;
        if (!_movement_sequence_select_client->wait_for_service(timeout))
            throw std::runtime_error("Movement sequence data server is not available");

        if (!_octomap_select_client->wait_for_service(timeout))
            throw std::runtime_error("Scene octomap data server is not available");

        if (!_trajectory_insert_client->wait_for_service(timeout))
            throw std::runtime_error("Trajectory data server is not available");
        
        // Create physics client handler and pass it to modules that needs it
        nlohmann::json parameters = helpers::commons::getParameter("robot");
        if (parameters.empty())
            throw std::runtime_error("Cannot read \"robot\" parameter from server");

        physics_client_handler::PhysicsClientHandler::SharedPtr pch;
        if (auto robot_info = helpers::commons::getRobotInfo())
        {
            pch = std::make_shared<physics_client_handler::PhysicsClientHandler>(*robot_info, get_logger());
        }
        else
            throw std::runtime_error("Cannot get robot information");

        _generate_path_handler = std::make_unique<generate_path::GeneratePath>(shared_from_this(), pch);
        _generate_trajectory_handler = std::make_unique<generate_trajectory::GenerateTrajectory>(shared_from_this());
        _spawn_collision_items_handler = std::make_unique<spawn_collision_items::SpawnCollisionItems>(shared_from_this(), pch);
        
        // Debugging publishers
        _generated_path_pub_debug = create_publisher<generate_path::GeneratedPath>("/debug/generated_path", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
        _generated_trajectory_pub_debug = create_publisher<trajectory_msgs::msg::JointTrajectory>("trajectory", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    }

    void MotionPlanning::_shutdown()
    {
        _action_server_motion_planning.reset();
        _trajectory_insert_client.reset();
        _octomap_select_client.reset();
        _movement_sequence_select_client.reset();
        _generate_path_handler.reset();
        _generate_trajectory_handler.reset();
        _spawn_collision_items_handler.reset();
        _generated_path_pub_debug.reset();
        _generated_trajectory_pub_debug.reset();
    }

    // ___Pose action___
    rclcpp_action::GoalResponse MotionPlanning::_handleGoal(const rclcpp_action::GoalUUID & /*uuid*/,
                                                            std::shared_ptr<const MotionPlanningAction::Goal> /*goal*/)
    {
        RCLCPP_INFO(get_logger(), "Goal acceped. Proceeding to execute goal pose");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse MotionPlanning::_handleCancel(const std::shared_ptr<GoalHandleMotionPlanningAction> /*goal_handle*/)
    {
        RCLCPP_INFO(get_logger(), "Goal to pose canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void MotionPlanning::_handleAccepted(const std::shared_ptr<GoalHandleMotionPlanningAction> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Goal to pose accepted");
        std::thread(std::bind(&MotionPlanning::_execute, this, std::placeholders::_1), goal_handle).detach();
    }

    void MotionPlanning::_execute(const std::shared_ptr<GoalHandleMotionPlanningAction> goal_handle)
    {
        helpers::Timer timer("Motion planning", get_logger());
        auto result = std::make_shared<MotionPlanningAction::Result>();

        // ----------------------------------------------------
        // Input processing - read from store
        // ----------------------------------------------------
        ReadData::SharedPtr read_data;
        try
        {
            read_data = _readData();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Error occured while reading data. Error: %s. Aborting...", e.what());
            goal_handle->abort(result);
            return;
        }

        // ----------------------------------------------------
        // Spawn collision items (items and collision octomap)
        // ----------------------------------------------------
        try
        {
            _spawn_collision_items_handler->spawnOctomap(read_data->octomap);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Error occured while spawning collision items. Error: %s. Aborting...", e.what());
            goal_handle->abort(result);
            return;
        }

        // ----------------------------------------------------
        // Generate path
        // ----------------------------------------------------
        generate_path::GeneratedPath::SharedPtr generated_path_segments;
        try
        {
            generated_path_segments = _generate_path_handler->generatePath(read_data->movement_sequence);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Error occured while generating path. Error: %s. Aborting...", e.what());
            goal_handle->abort(result);
            return;
        }

        if (_generated_path_pub_debug->get_subscription_count() > 0)
        {
            RCLCPP_DEBUG(get_logger(), "Publishing generated path for debugging");
            _generated_path_pub_debug->publish(*generated_path_segments);
        }

        size_t path_length = 0;
        for (auto &path_segment : generated_path_segments->path_segments)
            path_length += path_segment.points.size();
        RCLCPP_DEBUG(get_logger(), "Generated path has length of %ld", path_length);

        // ----------------------------------------------------
        // Generate trajectory
        // ----------------------------------------------------
        trajectory_msgs::msg::JointTrajectory::SharedPtr generated_trajectory;
        try
        {
            generated_trajectory = _generate_trajectory_handler->generateTrajectoryFromPath(generated_path_segments);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Error occured while generating trajectory. Error: %s. Aborting...", e.what());
            goal_handle->abort(result);
            return;
        }
        RCLCPP_DEBUG(get_logger(), "Generated trajectory has length of %ld", generated_trajectory->points.size());
        
        if (_generated_trajectory_pub_debug->get_subscription_count() > 0)
        {
            RCLCPP_DEBUG(get_logger(), "Publishing generated trajectory for debugging");
            _generated_trajectory_pub_debug->publish(*generated_trajectory);
        }

        // ----------------------------------------------------
        // Output processing - write to store
        // ----------------------------------------------------
        try
        {
            WriteData::SharedPtr write_data = std::make_shared<WriteData>();
            write_data->trajectory = generated_trajectory;
            _writeData(write_data);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Error occured while writing data. Error: %s. Aborting...", e.what());
            goal_handle->abort(result);
            return;
        }
        
        if (rclcpp::ok())
        {
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal to pose succeeded");
        }
        RCLCPP_INFO(get_logger(), "Generate to pose finished");
    }

    ReadData::SharedPtr MotionPlanning::_readData()
    {
        ReadData::SharedPtr read_data = std::make_shared<ReadData>();

        // Read movement sequence from data server
        auto movement_sequence_req = std::make_shared<MovementSequenceSelect::Request>();
        movement_sequence_req->time_stamp.data = 0.0; // Currently it does not matter
        auto movement_sequence_res = _movement_sequence_select_client->async_send_request(movement_sequence_req);
        if (movement_sequence_res.wait_for(1s) != std::future_status::ready)
            throw std::runtime_error("Cannot read movement sequence from server");

        read_data->movement_sequence = movement_sequence_res.get()->data;
        if (read_data->movement_sequence.size() == 0)
            throw std::runtime_error("There are no end effector poses in movement sequence");

        // Validate path types
        for (size_t i = 0; i < read_data->movement_sequence.size(); i++)
        {
            auto &el = read_data->movement_sequence[i];
            if (el.path_type != EndEffectorPose::PATH && el.path_type != EndEffectorPose::LINEAR && el.path_type != EndEffectorPose::ORIENTATION && el.path_type != EndEffectorPose::LO)
                throw std::runtime_error("Invalid path type specified for " + std::to_string(i + 1) + " end effector pose");
        }

        // Read octomap of the scene
        auto octomap_req = std::make_shared<OctomapSelect::Request>();
        octomap_req->time_stamp.data = 0.0; // Currently it does not matter
        auto octomap_res = _octomap_select_client->async_send_request(octomap_req);
        if (octomap_res.wait_for(1s) != std::future_status::ready)
            throw std::runtime_error("Cannot read scene octomap from server");

        auto ros_octomap = octomap_res.get()->data.octomap.scene_octomap;
        helpers::converters::rosPtcldtoPcl<pcl::PointXYZ>(ros_octomap, read_data->octomap);

        return read_data;
    }

    void MotionPlanning::_writeData(const WriteData::SharedPtr write_data)
    {
        if (!write_data->trajectory)
            throw std::runtime_error("Trajectory is not set");

        auto trajectory_req = std::make_shared<TrajectoryInsert::Request>();
        trajectory_req->time_stamp.data = 0.0; // Currently it does not matter
        trajectory_req->data = *write_data->trajectory;
        auto trajectory_res = _trajectory_insert_client->async_send_request(trajectory_req);
        if (trajectory_res.wait_for(1s) != std::future_status::ready)
            throw std::runtime_error("Cannot write trajectory to server");

        if (!trajectory_res.get()->result.data)
            throw std::runtime_error("Trajectory data was not saved successfully");
    }

} // namespace generate_path

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(motion_planning::MotionPlanning)
