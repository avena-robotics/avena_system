#include "generate_path/generate_path.hpp"

namespace generate_path
{
    GeneratePath::GeneratePath(const rclcpp::NodeOptions &options)
        : Node("generate_path", options)
    {
        helpers::commons::setLoggerLevelFromParameter(this);
        auto log_level = rcutils_logging_get_logger_level(get_logger().get_name());
        if (log_level == RCUTILS_LOG_SEVERITY_DEBUG)
            ompl::msg::setLogLevel(ompl::msg::LOG_DEBUG);
        else if (log_level == RCUTILS_LOG_SEVERITY_INFO)
            ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
        else if (log_level == RCUTILS_LOG_SEVERITY_WARN)
            ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
        else if (log_level == RCUTILS_LOG_SEVERITY_ERROR || log_level == RCUTILS_LOG_SEVERITY_FATAL)
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

        status = custom_interfaces::msg::Heartbeat::STOPPED;
        _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
    }

    GeneratePath::~GeneratePath()
    {
        shutDownNode();
    }

    void GeneratePath::initNode()
    {
        RCLCPP_DEBUG(get_logger(), "Initializing node");
        status = custom_interfaces::msg::Heartbeat::STARTING;
        if (_initialize() != ReturnCode::SUCCESS)
        {
            RCLCPP_WARN(get_logger(), "Error occured while initializing node");
            status = custom_interfaces::msg::Heartbeat::STOPPED;
            return;
        }
        status = custom_interfaces::msg::Heartbeat::RUNNING;
    }

    void GeneratePath::shutDownNode()
    {
        RCLCPP_DEBUG(get_logger(), "Shutting down node");
        status = custom_interfaces::msg::Heartbeat::STOPPING;
        if (_shutdown() != ReturnCode::SUCCESS)
            RCLCPP_ERROR(get_logger(), "Error occured when shutting down node");
        status = custom_interfaces::msg::Heartbeat::STOPPED;
    }

    ReturnCode GeneratePath::_initialize()
    {
        rclcpp::QoS latching_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        _joint_state_sub = create_subscription<sensor_msgs::msg::JointState>("arm_joint_states", 10,
                                                                             [this](const sensor_msgs::msg::JointState::SharedPtr joint_states_msg)
                                                                             {
                                                                                 std::lock_guard<std::mutex> lg(_current_joint_states_mtx);
                                                                                 //   RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Received joint states [message throttles with 0.1 sec]");
                                                                                 _current_joint_states = joint_states_msg;
                                                                             });
        _generated_path_pub = create_publisher<custom_interfaces::msg::GeneratedPath>("generated_path", latching_qos);
        _action_server_pose = rclcpp_action::create_server<GeneratePathPose>(
            this, "generate_path_pose",
            std::bind(&GeneratePath::_handleGoalPose, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&GeneratePath::_handleCancelPose, this, std::placeholders::_1),
            std::bind(&GeneratePath::_handleAcceptedPose, this, std::placeholders::_1));

        if (_getParametersFromServer() != ReturnCode::SUCCESS)
        {
            RCLCPP_WARN(get_logger(), "Cannot read parameters from server");
            return ReturnCode::FAILURE;
        }

        try
        {
            _physics_client_handler = std::make_shared<physics_client_handler::PhysicsClientHandler>(_robot_info, get_logger());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_STREAM(get_logger(), e.what());
            return ReturnCode::FAILURE;
        }

        try
        {
            _ik_engine = std::make_shared<inverse_kinematics::InverseKinematics>(_physics_client_handler, _robot_info, get_logger());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_STREAM(get_logger(), e.what());
            return ReturnCode::FAILURE;
        }

        // _updateJointLimits();

        // _scene_info = std::make_shared<SceneInfo>();
        // _scene_info->bullet_client = std::make_shared<bullet_client::b3RobotSimulatorClientAPI>();
        // RCLCPP_DEBUG(get_logger(), "Connecting to physics server");
        // bool connected = _scene_info->bullet_client->connect(eCONNECT_SHARED_MEMORY);
        // if (!connected)
        // {
        //     RCLCPP_WARN(get_logger(), "Cannot connect to physics server");
        //     return ReturnCode::FAILURE;
        // }
        // _scene_info->bullet_client->syncBodies();
        // RCLCPP_INFO_STREAM(get_logger(), "Connected to physics server successfully. API version: " << _scene_info->bullet_client->getAPIVersion());

        // if (_readSceneInfoFromPhysicsServer() != ReturnCode::SUCCESS)
        // {
        //     RCLCPP_WARN(get_logger(), "Cannot read info from physics server");
        //     return ReturnCode::FAILURE;
        // }

        // // Get transform to base link
        // if (auto robot_base_tf = helpers::vision::getTransformStamped("world", _robot_info.base_link_name))
        //     _robot_base_tf = *robot_base_tf;
        // else
        // {
        //     RCLCPP_WARN(get_logger(), "Cannot read transform for robot base");
        //     return ReturnCode::FAILURE;
        // }

        return ReturnCode::SUCCESS;
    }

    ReturnCode GeneratePath::_shutdown()
    {
        _joint_state_sub.reset();
        _generated_path_pub.reset();
        _action_server_pose.reset();
        _physics_client_handler.reset();
        // _ik_engine.reset();
        // _scene_info.reset();
        // _robot_info.limits.clear();
        return ReturnCode::SUCCESS;
    }

    // ___Pose action___
    rclcpp_action::GoalResponse GeneratePath::_handleGoalPose(const rclcpp_action::GoalUUID & /*uuid*/,
                                                              std::shared_ptr<const GeneratePathPose::Goal> /*goal*/)
    {
        RCLCPP_INFO(get_logger(), "Goal acceped. Proceeding to execute goal pose");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse GeneratePath::_handleCancelPose(const std::shared_ptr<GoalHandleGeneratePathPose> /*goal_handle*/)
    {
        RCLCPP_INFO(get_logger(), "Goal to pose canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void GeneratePath::_handleAcceptedPose(const std::shared_ptr<GoalHandleGeneratePathPose> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Goal to pose accepted");
        std::thread(std::bind(&GeneratePath::_executePose, this, std::placeholders::_1), goal_handle).detach();
    }

    void GeneratePath::_executePose(const std::shared_ptr<GoalHandleGeneratePathPose> goal_handle)
    {
        helpers::Timer timer("Generate path to pose", get_logger());
        auto result = std::make_shared<GeneratePathPose::Result>();

        // Save current state of joints to prevent data races
        sensor_msgs::msg::JointState::SharedPtr current_joint_states;
        {
            std::lock_guard<std::mutex> lg(_current_joint_states_mtx);
            if (!_current_joint_states)
            {
                RCLCPP_ERROR(get_logger(), "There is no joint values coming to module. Aborting...");
                _generated_path_pub->publish(custom_interfaces::msg::GeneratedPath());
                goal_handle->abort(result);
                return;
            }
            current_joint_states = std::make_shared<sensor_msgs::msg::JointState>(*_current_joint_states);
            _current_joint_states.reset();
        }

        RCLCPP_INFO(get_logger(), "Generating pose path");
        _physics_client_handler->cleanDebugItems();
        _physics_client_handler->syncWithPhysicsServer();

        if (!_physics_client_handler->isSceneValid())
        {
            RCLCPP_ERROR(get_logger(), "Scene in physics server is not valid. Aborting...");
            _generated_path_pub->publish(custom_interfaces::msg::GeneratedPath());
            goal_handle->abort(result);
            return;
        }

        auto current_joint_values = _getJointStatesFromTopic(current_joint_states);

        PathPlanningInput path_planning_input;
        // Save start and goal state and scene info
        path_planning_input.start_state = current_joint_values;
        path_planning_input.physics_client_handler = _physics_client_handler;
        path_planning_input.state_space_size = _robot_info.nr_joints;
        // path_planning_input.scene_info = _scene_info;
        // Setup constraints
        // path_planning_input.constraints = std::make_shared<Constraints>();
        // path_planning_input.constraints->contact_number_allowed = _contact_number_allowed;
        // path_planning_input.constraints->safety_distance = _safety_range;
        // for (const auto &limit : _robot_info.soft_limits)
        path_planning_input.limits = _robot_info.soft_limits;

        // Set all obstacles (there should be also all collision items estimated)
        // path_planning_input.constraints->obstacles.push_back(_table_idx);

        // Check initial state validity
        try
        {
            RCLCPP_DEBUG(get_logger(), "Validating initial state.");
            _validateArmConfiguration(path_planning_input.start_state);
        }
        catch(const std::runtime_error& e)
        {
            RCLCPP_ERROR_STREAM(get_logger(), e.what() << ". Aborting...");
            _generated_path_pub->publish(custom_interfaces::msg::GeneratedPath());
            goal_handle->abort(result);
            return;
        }
        
        // Inverse kinematics
        RCLCPP_DEBUG(get_logger(), "Inverse kinematics calculation");
        auto goal_end_effector_pose_ros = goal_handle.get()->get_goal()->end_effector_pose;
        helpers::converters::geometryToEigenAffine(goal_end_effector_pose_ros, path_planning_input.goal_end_effector_pose);
        _physics_client_handler->drawCoordinateAxes(path_planning_input.goal_end_effector_pose);
        _physics_client_handler->setJointStates(path_planning_input.start_state);
        path_planning_input.goal_state = _ik_engine->computeIk(path_planning_input.goal_end_effector_pose);

        if (path_planning_input.goal_state.size() == 0)
        {
            RCLCPP_ERROR(get_logger(), "Could not find valid arm configuration for provided end effector pose. Aborting...");
            _generated_path_pub->publish(custom_interfaces::msg::GeneratedPath());
            goal_handle->abort(result);
            return;
        }

        // Path planning
        // Set joint states back to initial state before planning
        _physics_client_handler->setJointStates(current_joint_values);

        Path out_path;
        Planner planer;
        if (planer.solve(path_planning_input, out_path) != ReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Error occured while planning path. Aborting...");
            _generated_path_pub->publish(custom_interfaces::msg::GeneratedPath());
            goal_handle->abort(result);
            return;
        }

        // // Set joint states back to initial state before planning
        // _setJointStates(current_joint_values);

        {
            // Set arm to last config for visualization
            auto last_arm_config = out_path.back();
            _physics_client_handler->setJointStates(last_arm_config);
        }

        // Output processing
        result->generated_path.header.stamp = now();
        result->generated_path.path_segments.resize(1);
        _convertPathSegmentToTrajectoryMsg(out_path, result->generated_path.path_segments[0]);

        _generated_path_pub->publish(result->generated_path);
        if (rclcpp::ok())
        {
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal to pose succeeded");
        }
        RCLCPP_INFO(get_logger(), "Generate to pose finished");
    }

    void GeneratePath::_validateArmConfiguration(const ArmConfiguration &joint_state)
    {
        _physics_client_handler->setJointStates(joint_state);
        if (_checkJointLimits(joint_state, _robot_info.limits) != ReturnCode::SUCCESS)
            throw std::runtime_error("Invalid state. Joint states are outside of limits");

        if (_physics_client_handler->inCollision())
            throw std::runtime_error("Invalid state. Robot is in collision with scene or itself");
    }

    ReturnCode GeneratePath::_checkJointLimits(const ArmConfiguration &joint_states, const std::vector<Limits> &joint_limits)
    {
        RCLCPP_DEBUG(get_logger(), "Check whether joint states are in limits");
        if (joint_states.size() != joint_limits.size())
        {
            RCLCPP_WARN_STREAM(get_logger(), "Amount of joint states (" << joint_states.size() << ") is different than amount of joint limits (" << joint_limits.size() << ")");
            return ReturnCode::FAILURE;
        }
        ReturnCode error_code = ReturnCode::SUCCESS;
        for (size_t i = 0; i < joint_states.size(); ++i)
        {
            if (joint_states[i] < joint_limits[i].lower || joint_states[i] > joint_limits[i].upper)
            {
                RCLCPP_DEBUG_STREAM(get_logger(), "Joint: " << i + 1 << ": value: " << joint_states[i] << ", limits: (" << joint_limits[i].lower << ", " << joint_limits[i].upper << ") [outside of limits]");
                error_code = ReturnCode::FAILURE;
            }
            else
                RCLCPP_DEBUG_STREAM(get_logger(), "Joint: " << i + 1 << ": value: " << joint_states[i] << ", limits: (" << joint_limits[i].lower << ", " << joint_limits[i].upper << ")");
        }
        return error_code;
    }

    // double GeneratePath::_calculateDistanceEndEffectorPosToGoalPos(const Eigen::Affine3d &end_effector_pose, const Eigen::Affine3d &goal_end_effector_pose)
    // {
    //     Eigen::Vector3d ee_position(end_effector_pose.translation());
    //     Eigen::Vector3d goal_ee_position(goal_end_effector_pose.translation());
    //     auto ee_position_dist = (ee_position - goal_ee_position).norm();
    //     return ee_position_dist;
    // }

    // double GeneratePath::_calculateDistanceEndEffectorOrienToGoalOrien(const Eigen::Affine3d &end_effector_pose, const Eigen::Affine3d &goal_end_effector_pose)
    // {
    //     Eigen::Quaterniond ee_orientation(end_effector_pose.rotation());
    //     Eigen::Quaterniond goal_ee_orientation(goal_end_effector_pose.rotation());
    //     auto angle = ee_orientation.angularDistance(goal_ee_orientation);
    //     return std::abs(angle);
    // }

    ArmConfiguration GeneratePath::_getJointStatesFromTopic(const sensor_msgs::msg::JointState::SharedPtr &joint_states)
    {
        if (joint_states->position.size() != joint_states->name.size())
            return {};

        ArmConfiguration current_joint_states;
        for (size_t i = 0; i < joint_states->position.size(); ++i)
        {
            const auto joint_name = joint_states->name[i];
            auto it = std::find(_robot_info.joint_names.begin(), _robot_info.joint_names.end(), joint_name);
            if (it != _robot_info.joint_names.end())
                current_joint_states.push_back(joint_states->position[i]);
        }

        return current_joint_states;
    }

    ReturnCode GeneratePath::_getParametersFromServer()
    {
        RCLCPP_INFO_ONCE(get_logger(), "Reading parameters from the server");

        nlohmann::json parameters = helpers::commons::getParameter("robot");
        if (parameters.empty())
            return ReturnCode::FAILURE;

        const std::string working_side = parameters["working_side"];
        if (auto robot_info = helpers::commons::getRobotInfo(working_side))
            _robot_info = *robot_info;
        else
            return ReturnCode::FAILURE;

        // Debug display of joint limits
        std::stringstream ss;
        ss << std::endl
           << "Limits for path planning (soft limits):" << std::endl;
        for (size_t i = 0; i < _robot_info.soft_limits.size(); ++i)
            ss << "  joint " << i + 1 << ": (" << _robot_info.soft_limits[i].lower << ", " << _robot_info.soft_limits[i].upper << ")" << std::endl;

        ss << "Limits for (hard limits):" << std::endl;
        for (size_t i = 0; i < _robot_info.limits.size(); ++i)
            ss << "  joint " << i + 1 << ": (" << _robot_info.limits[i].lower << ", " << _robot_info.limits[i].upper << ")" << std::endl;
        RCLCPP_DEBUG(get_logger(), ss.str());

        RCLCPP_INFO(get_logger(), "Parameters read successfully...");
        return ReturnCode::SUCCESS;
    }

    void GeneratePath::_convertPathSegmentToTrajectoryMsg(const std::vector<ArmConfiguration> &path, trajectory_msgs::msg::JointTrajectory &path_segment)
    {
        path_segment.points.resize(path.size());
        path_segment.joint_names = _robot_info.joint_names;
        for (size_t i = 0; i < path.size(); ++i)
        {
            path_segment.points[i].positions = path[i];
        }
    }

} // namespace generate_path

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(generate_path::GeneratePath)
