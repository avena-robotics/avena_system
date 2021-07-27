#include "generate_path/generate_path.hpp"

namespace generate_path
{

    GeneratePath::GeneratePath(const rclcpp::NodeOptions &options)
        : Node("generate_path", options)
    {
        helpers::commons::setLoggerLevel(get_logger(), "debug");
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
                                                                                 //  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Received joint states [message throttles with 0.1 sec]");
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

        _scene_info = std::make_shared<SceneInfo>();
        _scene_info->bullet_client = std::make_shared<bullet_client::b3RobotSimulatorClientAPI>();
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
        auto result = std::make_shared<GeneratePathPose::Result>();
        
        // Save current state of joints to prevent data races
        sensor_msgs::msg::JointState::SharedPtr current_joint_states;
        {
            std::lock_guard<std::mutex> lg(_current_joint_states_mtx);
            current_joint_states = std::make_shared<sensor_msgs::msg::JointState>(*_current_joint_states);
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
        for (const auto &bound : _robot_info.bounds)
        {
            path_planning_input.constraints->low_bounds.push_back(bound.bounds_low);
            path_planning_input.constraints->high_bounds.push_back(bound.bounds_high);
        }
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

        auto end_effector_pose = goal_handle.get()->get_goal()->end_effector_pose;
        path_planning_input.goal_state = _calculateGoalStateFromEndEffectorPose(end_effector_pose, current_joint_states);

        // Check goal state validity
        _setJointStates(path_planning_input.goal_state);
        if (Planner::calculateContactPointsAmount(path_planning_input) > _contact_number_allowed)
        {
            RCLCPP_ERROR(get_logger(), "Invalid final state. Robot is in collision with scene or itself. Aborting...");
            _generated_path_pub->publish(custom_interfaces::msg::GeneratedPath());
            goal_handle->abort(result);
            return;
        }

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

        // // Set arm to last config for visualization
        // {
        //     auto last_arm_config = out_path.back();
        //     _setJointStates(last_arm_config);
        // }

        // Output processing
        custom_interfaces::msg::GeneratedPath generated_path;
        generated_path.header.stamp = now();
        generated_path.path_segments.resize(1);
        _convertPathSegmentToTrajectoryMsg(out_path, generated_path.path_segments[0]);

        _generated_path_pub->publish(generated_path);
        if (rclcpp::ok())
        {
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal to pose succeeded");
        }
        RCLCPP_INFO(get_logger(), "Generate to pose finished");
    }

    ArmConfiguration GeneratePath::_calculateGoalStateFromEndEffectorPose(const geometry_msgs::msg::Pose &end_effector_pose,
                                                                          const sensor_msgs::msg::JointState::SharedPtr & /*current_joint_states*/)
    {
        b3RobotSimulatorInverseKinematicArgs ik_args;
        ik_args.m_bodyUniqueId = _scene_info->robot_idx;
        ik_args.m_endEffectorLinkIndex = _scene_info->end_effector_idx;
        ik_args.m_endEffectorTargetPosition[0] = end_effector_pose.position.x;
        ik_args.m_endEffectorTargetPosition[1] = end_effector_pose.position.y;
        ik_args.m_endEffectorTargetPosition[2] = end_effector_pose.position.z;
        ik_args.m_endEffectorTargetOrientation[0] = end_effector_pose.orientation.x;
        ik_args.m_endEffectorTargetOrientation[1] = end_effector_pose.orientation.y;
        ik_args.m_endEffectorTargetOrientation[2] = end_effector_pose.orientation.z;
        ik_args.m_endEffectorTargetOrientation[3] = end_effector_pose.orientation.w;
        ik_args.m_numDegreeOfFreedom = _robot_info.nr_joints;

        ik_args.m_lowerLimits.resize(_robot_info.bounds.size());
        ik_args.m_upperLimits.resize(_robot_info.bounds.size());
        ik_args.m_jointRanges.resize(_robot_info.bounds.size());
        ik_args.m_restPoses.resize(_robot_info.bounds.size());
        ik_args.m_jointDamping.resize(_robot_info.bounds.size());
        for (size_t i = 0; i < _robot_info.bounds.size(); ++i)
        {
            ik_args.m_lowerLimits[i] = _robot_info.bounds[i].bounds_low;
            ik_args.m_upperLimits[i] = _robot_info.bounds[i].bounds_high;
            ik_args.m_jointRanges[i] = std::abs(_robot_info.bounds[i].bounds_high) + std::abs(_robot_info.bounds[i].bounds_low);
            ik_args.m_restPoses[i] = 0;
            ik_args.m_jointDamping[i] = 0.1;
        }
        ik_args.m_jointDamping[5] = 1;

        ik_args.m_flags |= B3_HAS_IK_TARGET_ORIENTATION;
        ik_args.m_flags |= B3_HAS_NULL_SPACE_VELOCITY;

        // ik_args.m_currentJointPositions.clear();
        // for (size_t i = 0; i < _robot_info.nr_joints; ++i)
        //     ik_args.m_currentJointPositions.push_back(current_joint_states->position[i]);
        // ik_args.m_flags |= B3_HAS_CURRENT_POSITIONS;

        // RCLCPP_ERROR_STREAM(get_logger(), "ik_args.m_flags: " << ik_args.m_flags);

        //////////////////////////////////////////////////////////
        // End effector pose visualization
        Eigen::Affine3f ee_aff;
        auto ros_ee_aff = end_effector_pose;
        helpers::converters::geometryToEigenAffine(ros_ee_aff, ee_aff);

        Eigen::Vector3f ee_pos(ee_aff.translation());
        Eigen::Quaternionf ee_quat(ee_aff.rotation());
        Eigen::Vector3f x_shift = ee_quat.toRotationMatrix().col(0) * 0.2;
        Eigen::Vector3f ee_pos_x = ee_pos + x_shift;

        Eigen::Vector3f y_shift = ee_quat.toRotationMatrix().col(1) * 0.2;
        Eigen::Vector3f ee_pos_y = ee_pos + y_shift;

        Eigen::Vector3f z_shift = ee_quat.toRotationMatrix().col(2) * 0.2;
        Eigen::Vector3f ee_pos_z = ee_pos + z_shift;

        b3RobotSimulatorAddUserDebugLineArgs LINE_ARGS;
        LINE_ARGS.m_colorRGB[0] = 1;
        LINE_ARGS.m_colorRGB[1] = 0;
        LINE_ARGS.m_colorRGB[2] = 0;
        btVector3 from(ros_ee_aff.position.x, ros_ee_aff.position.y, ros_ee_aff.position.z);
        btVector3 to_x(ee_pos_x.x(), ee_pos_x.y(), ee_pos_x.z());
        _scene_info->bullet_client->addUserDebugLine(from, to_x, LINE_ARGS);
        LINE_ARGS.m_colorRGB[0] = 0;
        LINE_ARGS.m_colorRGB[1] = 1;
        LINE_ARGS.m_colorRGB[2] = 0;
        btVector3 to_y(ee_pos_y.x(), ee_pos_y.y(), ee_pos_y.z());
        _scene_info->bullet_client->addUserDebugLine(from, to_y, LINE_ARGS);
        LINE_ARGS.m_colorRGB[0] = 0;
        LINE_ARGS.m_colorRGB[1] = 0;
        LINE_ARGS.m_colorRGB[2] = 1;
        btVector3 to_z(ee_pos_z.x(), ee_pos_z.y(), ee_pos_z.z());
        _scene_info->bullet_client->addUserDebugLine(from, to_z, LINE_ARGS);
        //////////////////////////////////////////////////////////

        b3RobotSimulatorInverseKinematicsResults ik_results;
        _scene_info->bullet_client->calculateIK(ik_args, ik_results);

        ArmConfiguration goal_configuration(_robot_info.nr_joints);
        for (size_t i = 0; i < goal_configuration.size(); i++)
            goal_configuration[i] = ik_results.m_calculatedJointPositions[i];

        return goal_configuration;
    }

    ReturnCode GeneratePath::_readSceneInfoFromPhysicsServer()
    {
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

    void GeneratePath::_convertPathSegmentToTrajectoryMsg(const std::vector<ArmConfiguration> &path, trajectory_msgs::msg::JointTrajectory &path_segment)
    {
        path_segment.points.resize(path.size());
        path_segment.joint_names = _robot_info.joint_names;
        for (size_t i = 0; i < path.size(); ++i)
        {
            path_segment.points[i].positions = std::vector<double>(path[i].begin(), path[i].end());
        }
    }

} // namespace generate_path

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(generate_path::GeneratePath)
