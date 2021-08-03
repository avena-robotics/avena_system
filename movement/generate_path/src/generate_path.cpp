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
        _joint_state_sub = create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,
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

        _updateJointLimits();

        _scene_info = std::make_shared<SceneInfo>();
        _scene_info->bullet_client = std::make_shared<bullet_client::b3RobotSimulatorClientAPI>();
        RCLCPP_DEBUG(get_logger(), "Connecting to physics server");
        bool connected = _scene_info->bullet_client->connect(eCONNECT_SHARED_MEMORY);
        if (!connected)
        {
            RCLCPP_WARN(get_logger(), "Cannot connect to physics server");
            return ReturnCode::FAILURE;
        }
        _scene_info->bullet_client->syncBodies();
        RCLCPP_INFO_STREAM(get_logger(), "Connected to physics server successfully. API version: " << _scene_info->bullet_client->getAPIVersion());

        if (_readSceneInfoFromPhysicsServer() != ReturnCode::SUCCESS)
        {
            RCLCPP_WARN(get_logger(), "Cannot read info from physics server");
            return ReturnCode::FAILURE;
        }

        // Get transform to base link
        if (auto robot_base_tf = helpers::vision::getTransformStamped("world", _robot_info.base_link_name))
            _robot_base_tf = *robot_base_tf;
        else
        {
            RCLCPP_WARN(get_logger(), "Cannot read transform for robot base");
            return ReturnCode::FAILURE;
        }

        return ReturnCode::SUCCESS;
    }

    ReturnCode GeneratePath::_shutdown()
    {
        _joint_state_sub.reset();
        _generated_path_pub.reset();
        _action_server_pose.reset();
        if (_scene_info->bullet_client->isConnected())
            _scene_info->bullet_client->disconnect();
        _scene_info.reset();
        _robot_info.limits.clear();
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
        _scene_info->bullet_client->removeAllUserDebugItems();
        _scene_info->bullet_client->syncBodies();

        auto current_joint_values = _getJointStatesFromTopic(current_joint_states);

        PathPlanningInput path_planning_input;
        // Save start and goal state and scene info
        path_planning_input.start_state = current_joint_values;
        path_planning_input.scene_info = _scene_info;
        // Setup constraints
        path_planning_input.constraints = std::make_shared<Constraints>();
        path_planning_input.constraints->contact_number_allowed = _contact_number_allowed;
        path_planning_input.constraints->safety_distance = _safety_range;
        for (const auto &limit : _robot_info.limits)
            path_planning_input.constraints->limits.push_back(Limits(limit.lower, limit.upper));

        // Set all obstacles (there should be also all collision items estimated)
        path_planning_input.constraints->obstacles.push_back(_table_idx);

        // Check initial state validity
        _setJointStates(path_planning_input.start_state);
        if (Planner::calculateContactPointsAmount(path_planning_input) > _contact_number_allowed)
        {
            RCLCPP_ERROR(get_logger(), "Invalid initial state. Robot is in collision with scene or itself. Aborting...");
            _generated_path_pub->publish(custom_interfaces::msg::GeneratedPath());
            goal_handle->abort(result);
            return;
        }

        auto goal_end_effector_pose_ros = goal_handle.get()->get_goal()->end_effector_pose;
        Eigen::Affine3d goal_end_effector_pose;
        helpers::converters::geometryToEigenAffine(goal_end_effector_pose_ros, goal_end_effector_pose);
        path_planning_input.goal_end_effector_pose = goal_end_effector_pose;
        _drawCoordinateAxes(goal_end_effector_pose);

        std::string compute_ik_error_message;
        std::tie(path_planning_input.goal_state, compute_ik_error_message) = _calculateIK(path_planning_input);

        if (path_planning_input.goal_state.size() == 0)
        {
            RCLCPP_ERROR(get_logger(), "Could not find valid arm configuration for set end effector pose. Error message: " + compute_ik_error_message + ". Aborting...");
            _generated_path_pub->publish(custom_interfaces::msg::GeneratedPath());
            goal_handle->abort(result);
            return;
        }

        // path_planning_input.goal_state = goal_configuration;
        // Set joint states back to initial state before planning
        _setJointStates(current_joint_values);

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

        // Set arm to last config for visualization
        {
            auto last_arm_config = out_path.back();
            _setJointStates(last_arm_config);
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

    ReturnCode GeneratePath::_validateConfiguration(const PathPlanningInput &path_planning_input, const ArmConfiguration &joint_state, std::string &error_message)
    {
        _setJointStates(joint_state);

        auto end_effector_pose = _getEndEffectorPose();

        if (_checkJointLimits(joint_state, _robot_info.limits) != ReturnCode::SUCCESS)
        {
            // RCLCPP_DEBUG(get_logger(), "Invalid final state. Joint states are outside of limits. Checking another configuration...");
            error_message = "Invalid final state. Joint states are outside of limits";
            return ReturnCode::FAILURE;
        }

        auto distance_ee_to_goal = GeneratePath::_calculateDistanceEndEffectorPosToGoalPos(end_effector_pose, path_planning_input.goal_end_effector_pose);
        auto distance_ee_to_goal_orien = GeneratePath::_calculateDistanceEndEffectorOrienToGoalOrien(end_effector_pose, path_planning_input.goal_end_effector_pose);
        RCLCPP_DEBUG_STREAM(get_logger(), "Distance from end effector to goal position: " << distance_ee_to_goal << " [m]");
        RCLCPP_DEBUG_STREAM(get_logger(), "Distance from end effector to goal orientation: " << distance_ee_to_goal_orien << " [rad]");
        if (distance_ee_to_goal > _end_effector_position_offset)
        {
            // RCLCPP_ERROR_STREAM(get_logger(), "Invalid final state. End effector is in invalid position. Distance to goal: " << distance_ee_to_goal << " [m]. Checking another configuration...");
            error_message = "Invalid final state. End effector is in invalid position. Distance to goal: " + std::to_string(distance_ee_to_goal) + " [m].";
            return ReturnCode::FAILURE;
        }
        if (distance_ee_to_goal_orien > _end_effector_orientation_offset)
        {
            // RCLCPP_ERROR_STREAM(get_logger(), "Invalid final state. End effector is in invalid position. Distance to goal orientation: " << distance_ee_to_goal_orien << " [rad].");
            error_message = "Invalid final state. End effector is in invalid position. Distance to goal orientation: " + std::to_string(distance_ee_to_goal_orien) + " [rad].";
            return ReturnCode::FAILURE;
        }

        if (Planner::calculateContactPointsAmount(path_planning_input) > _contact_number_allowed)
        {
            // RCLCPP_ERROR(get_logger(), "Invalid final state. Robot is in collision with scene or itself. Aborting...");
            error_message = "Invalid final state. Robot is in collision with scene or itself.";
            return ReturnCode::FAILURE;
        }
        return ReturnCode::SUCCESS;
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

    Eigen::Affine3d GeneratePath::_getEndEffectorPose()
    {
        b3LinkState link_state;
        if (!_scene_info->bullet_client->getLinkState(_scene_info->robot_idx, _scene_info->end_effector_idx, 0, 0, &link_state))
            return Eigen::Translation3d(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()) * Eigen::Quaterniond::Identity();

        Eigen::Translation3d ee_position(link_state.m_worldLinkFramePosition[0], link_state.m_worldLinkFramePosition[1], link_state.m_worldLinkFramePosition[2]);
        Eigen::Quaterniond ee_orientation(link_state.m_worldLinkFrameOrientation[3], link_state.m_worldLinkFrameOrientation[0], link_state.m_worldLinkFrameOrientation[1], link_state.m_worldLinkFrameOrientation[2]);
        return ee_position * ee_orientation;
    }

    double GeneratePath::_calculateDistanceEndEffectorPosToGoalPos(const Eigen::Affine3d &end_effector_pose, const Eigen::Affine3d &goal_end_effector_pose)
    {
        Eigen::Vector3d ee_position(end_effector_pose.translation());
        Eigen::Vector3d goal_ee_position(goal_end_effector_pose.translation());
        auto ee_position_dist = (ee_position - goal_ee_position).norm();
        return ee_position_dist;
    }

    double GeneratePath::_calculateDistanceEndEffectorOrienToGoalOrien(const Eigen::Affine3d &end_effector_pose, const Eigen::Affine3d &goal_end_effector_pose)
    {
        Eigen::Quaterniond ee_orientation(end_effector_pose.rotation());
        Eigen::Quaterniond goal_ee_orientation(goal_end_effector_pose.rotation());
        auto angle = ee_orientation.angularDistance(goal_ee_orientation);
        return std::abs(angle);
    }

    std::tuple<ArmConfiguration, std::string> GeneratePath::_calculateIK(const PathPlanningInput &path_planning_input)
    {
        helpers::Timer timer(__func__, get_logger());
        // TODO: Discuss convention for axes of end effector
        // Get rotation of end effector
        IkReal eerot[9];
        auto quat = Eigen::Quaterniond(path_planning_input.goal_end_effector_pose.rotation());
        auto ee_rot_matrix = quat.normalized().toRotationMatrix();
        auto rotation_z = Eigen::AngleAxisd(-M_PI_4, ee_rot_matrix.col(2));
        auto rotation_y = Eigen::AngleAxisd(M_PI_2, ee_rot_matrix.col(1));
        auto rotation_x = Eigen::AngleAxisd(-M_PI_2, ee_rot_matrix.col(0));
        quat = rotation_x * rotation_y * rotation_z * quat;
        auto rot_matrix = quat.normalized().toRotationMatrix();
        eerot[0] = rot_matrix(0);
        eerot[1] = rot_matrix(3);
        eerot[2] = rot_matrix(6);
        eerot[3] = rot_matrix(1);
        eerot[4] = rot_matrix(4);
        eerot[5] = rot_matrix(7);
        eerot[6] = rot_matrix(2);
        eerot[7] = rot_matrix(5);
        eerot[8] = rot_matrix(8);

        // Get translation of end effector
        IkReal eetrans[3];
        auto trans_vec = Eigen::Vector3d(path_planning_input.goal_end_effector_pose.translation());
        eetrans[0] = trans_vec(0) - _robot_base_tf.transform.translation.x;
        eetrans[1] = trans_vec(1) - _robot_base_tf.transform.translation.y;
        eetrans[2] = trans_vec(2) - _robot_base_tf.transform.translation.z;

        const auto free_joint_idx = GetFreeIndices();
        const double joint_state_increment = 0.1;
        const auto free_joint_limits = _robot_info.limits[free_joint_idx[0]];
        const double middle_range = (free_joint_limits.lower + free_joint_limits.upper) / 2;
        double lower_joint_state = middle_range;
        double upper_joint_state = middle_range + joint_state_increment;
        
        size_t amount_of_samples = 0; 
        std::string error_msg = "success";
        ArmConfiguration goal_configuration;

        auto check_single_ik = [=, &amount_of_samples](const double &free_joint_val) mutable -> std::tuple<ArmConfiguration, std::string>
        {
            std::string error_msg = "success";
            ArmConfiguration goal_configuration;

            std::vector<IkReal> vfree = {free_joint_val};
            IkSolutionList<IkReal> solutions;
            if (ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions))
            {
                RCLCPP_DEBUG_STREAM(get_logger(), "Found " << solutions.GetNumSolutions() << " IK solutions");
                std::vector<IkReal> solvalues(GetNumJoints());
                for (std::size_t i = 0; i < solutions.GetNumSolutions(); ++i)
                {
                    amount_of_samples++;

                    const IkSolutionBase<IkReal> &sol = solutions.GetSolution(i);
                    std::vector<IkReal> vsolfree(sol.GetFree().size());
                    sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);

                    // Validate calculated configuration
                    if (_validateConfiguration(path_planning_input, solvalues, error_msg) != ReturnCode::SUCCESS)
                        continue;

                    goal_configuration = solvalues;

                    // cv::Mat img = cv::Mat::ones(100, 100, CV_8UC1) * 255;
                    // cv::putText(img,                                                                       //target image
                    //             std::to_string(i + 1) + "_" + std::to_string(solutions.GetNumSolutions()), //text
                    //             cv::Point(10, img.rows / 2),                                               //top-left position
                    //             cv::FONT_HERSHEY_DUPLEX,
                    //             0.5,
                    //             CV_RGB(118, 185, 0), //font color
                    //             2);
                    // cv::imshow("kek", img);
                    // cv::waitKey();
                    // printf("\n");
                    break;
                }
            }
            return std::make_tuple(goal_configuration, error_msg);
        };

        while (true)
        {
            if (lower_joint_state < free_joint_limits.lower && upper_joint_state > free_joint_limits.upper)
                break;

            if (lower_joint_state >= free_joint_limits.lower)
            {
                std::tie(goal_configuration, error_msg) = check_single_ik(lower_joint_state);
                if (goal_configuration.size() > 0)
                    break;
            }

            if (upper_joint_state <= free_joint_limits.upper)
            {
                std::tie(goal_configuration, error_msg) = check_single_ik(upper_joint_state);
                if (goal_configuration.size() > 0)
                    break;
            }

            lower_joint_state -= joint_state_increment;
            upper_joint_state += joint_state_increment;
        }

        // RCLCPP_WARN_STREAM(get_logger(), amount_of_samples);

        return std::make_tuple(goal_configuration, error_msg);
    }

    ReturnCode GeneratePath::_readSceneInfoFromPhysicsServer()
    {
        RCLCPP_DEBUG(get_logger(), "Reading info about objects from physics server");
        int num_bodies = _scene_info->bullet_client->getNumBodies();
        for (int body_id = 0; body_id < num_bodies; ++body_id)
        {
            int body_unique_id = _scene_info->bullet_client->getBodyUniqueId(body_id);
            b3BodyInfo body_info;
            _scene_info->bullet_client->getBodyInfo(body_unique_id, &body_info);
            if (std::strcmp(body_info.m_bodyName, _robot_info.robot_name.c_str()) == 0)
            {
                _scene_info->robot_idx = body_unique_id;
                int num_joints = _scene_info->bullet_client->getNumJoints(body_unique_id);
                for (int joint_idx = 0; joint_idx < num_joints; ++joint_idx)
                {
                    b3JointInfo joint_info;
                    _scene_info->bullet_client->getJointInfo(body_unique_id, joint_idx, &joint_info);
                    if (std::strcmp(joint_info.m_linkName, _robot_info.connection.c_str()) == 0)
                        _scene_info->end_effector_idx = joint_idx;

                    if (joint_info.m_jointType != JointType::eFixedType)
                    {
                        auto joint_name_it = std::find(_robot_info.joint_names.begin(), _robot_info.joint_names.end(), std::string(joint_info.m_jointName));
                        if (joint_name_it != _robot_info.joint_names.end())
                            _scene_info->joint_handles.push_back(joint_idx);
                    }
                }
            }
            else
                _table_idx = body_unique_id;
        }
        RCLCPP_INFO_STREAM(get_logger(), "End effector ID: " << _scene_info->end_effector_idx);
        return ReturnCode::SUCCESS;
    }

    ArmConfiguration GeneratePath::_getJointStatesFromTopic(const sensor_msgs::msg::JointState::SharedPtr &joint_states)
    {
        ArmConfiguration current_joint_states;
        int num_joints = _scene_info->bullet_client->getNumJoints(_scene_info->robot_idx);
        for (int joint_idx = 0; joint_idx < num_joints; ++joint_idx)
        {
            b3JointInfo joint_info;
            _scene_info->bullet_client->getJointInfo(_scene_info->robot_idx, joint_idx, &joint_info);
            if (joint_info.m_jointType != JointType::eFixedType)
            {
                auto joint_name_it = std::find(joint_states->name.begin(), joint_states->name.end(), std::string(joint_info.m_jointName));
                auto joint_position_idx = std::distance(joint_states->name.begin(), joint_name_it);
                float joint_val = joint_states->position[joint_position_idx];
                current_joint_states.push_back(joint_val);
                // _scene_info->bullet_client->resetJointState(_scene_info->robot_idx, joint_idx, joint_val);
            }
        }
        return current_joint_states;
    }

    void GeneratePath::_setJointStates(const ArmConfiguration &joint_states)
    {
        for (int i = 0; i < _scene_info->end_effector_idx; i++)
            _scene_info->bullet_client->resetJointState(_scene_info->robot_idx, i, joint_states[i]);
    }

    ReturnCode GeneratePath::_getParametersFromServer()
    {
        RCLCPP_INFO_ONCE(get_logger(), "Reading parameters from the server");

        nlohmann::json parameters = helpers::commons::getParameter("robot");
        if (parameters.empty())
            return ReturnCode::FAILURE;

        const std::string working_side = parameters["working_side"];
        _robot_info = helpers::commons::getRobotInfo(working_side);

        RCLCPP_INFO(get_logger(), "Parameters read successfully...");
        return ReturnCode::SUCCESS;
    }

    void GeneratePath::_updateJointLimits()
    {
        RCLCPP_DEBUG(get_logger(), "Update joint limits by using soft limits");
        const double range_tighten_coeff = 0.95; // values from 0.0 - 1.0 how much scale down range for joint limits
        for (size_t i = 0; i < _robot_info.limits.size(); ++i)
        {
            auto range_middle = (_robot_info.limits[i].lower + _robot_info.limits[i].upper) / 2.0;
            auto range_middle_to_limit_dist = _robot_info.limits[i].upper - range_middle;
            auto offset = (range_middle_to_limit_dist * (1.0 - range_tighten_coeff)) / 2.0;
            // _robot_info.limits.push_back(Limits(_robot_info.limits[i].lower + offset * 2, _robot_info.limits[i].upper - offset * 2));
            _robot_info.limits[i].lower += offset;
            _robot_info.limits[i].upper -= offset;
        }

        std::stringstream ss;
        ss << std::endl
           << "Limits for path planning:" << std::endl;
        for (size_t i = 0; i < _robot_info.limits.size(); ++i)
            ss << "  joint " << i + 1 << ": (" << _robot_info.limits[i].lower << ", " << _robot_info.limits[i].upper << ")" << std::endl;

        ss << "Limits for IK:" << std::endl;
        for (size_t i = 0; i < _robot_info.limits.size(); ++i)
            ss << "  joint " << i + 1 << ": (" << _robot_info.limits[i].lower << ", " << _robot_info.limits[i].upper << ")" << std::endl;
        RCLCPP_DEBUG(get_logger(), ss.str());
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

    void GeneratePath::_drawCoordinateAxes(const Eigen::Affine3d &pose)
    {
        Eigen::Vector3d position(pose.translation());
        Eigen::Quaterniond orientation(pose.rotation());
        Eigen::Vector3d x_shift = orientation.toRotationMatrix().col(0) * 0.2;
        Eigen::Vector3d pos_x = position + x_shift;

        Eigen::Vector3d y_shift = orientation.toRotationMatrix().col(1) * 0.2;
        Eigen::Vector3d pos_y = position + y_shift;

        Eigen::Vector3d z_shift = orientation.toRotationMatrix().col(2) * 0.2;
        Eigen::Vector3d pos_z = position + z_shift;

        b3RobotSimulatorAddUserDebugLineArgs LINE_ARGS;
        LINE_ARGS.m_colorRGB[0] = 1;
        LINE_ARGS.m_colorRGB[1] = 0;
        LINE_ARGS.m_colorRGB[2] = 0;
        btVector3 from(position.x(), position.y(), position.z());
        btVector3 to_x(pos_x.x(), pos_x.y(), pos_x.z());
        _scene_info->bullet_client->addUserDebugLine(from, to_x, LINE_ARGS);
        LINE_ARGS.m_colorRGB[0] = 0;
        LINE_ARGS.m_colorRGB[1] = 1;
        LINE_ARGS.m_colorRGB[2] = 0;
        btVector3 to_y(pos_y.x(), pos_y.y(), pos_y.z());
        _scene_info->bullet_client->addUserDebugLine(from, to_y, LINE_ARGS);
        LINE_ARGS.m_colorRGB[0] = 0;
        LINE_ARGS.m_colorRGB[1] = 0;
        LINE_ARGS.m_colorRGB[2] = 1;
        btVector3 to_z(pos_z.x(), pos_z.y(), pos_z.z());
        _scene_info->bullet_client->addUserDebugLine(from, to_z, LINE_ARGS);
    }

} // namespace generate_path

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(generate_path::GeneratePath)
