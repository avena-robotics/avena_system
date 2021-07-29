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

        _updateJointLimits();

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
        _ik_joint_limits.clear();
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

        auto end_effector_pose_ros = goal_handle.get()->get_goal()->end_effector_pose;
        Eigen::Affine3d end_effector_pose;
        helpers::converters::geometryToEigenAffine(end_effector_pose_ros, end_effector_pose);
        path_planning_input.goal_state = _calculateGoalStateFromEndEffectorPose(end_effector_pose);

        // // int argc;
        // // char **argv;
        // std::vector<IkReal> vfree = {0};

        // RCLCPP_WARN_STREAM(get_logger(), "Free parameters: " << vfree.size());
        // RCLCPP_WARN_STREAM(get_logger(), "Value: " << vfree[0]);

        // IkReal eerot[9];
        // Eigen::Affine3f ee_aff;
        // helpers::converters::geometryToEigenAffine(end_effector_pose, ee_aff);
        // auto quat = Eigen::Quaternionf(ee_aff.rotation());
        // // auto quat = Eigen::Quaternionf::Identity();
        // quat.normalize();
        // auto rot_matrix = quat.toRotationMatrix();
        // for (size_t i = 0; i < 9; ++i)
        // {
        //     eerot[i] = rot_matrix(i);
        //     RCLCPP_WARN_STREAM(get_logger(), eerot[i]);
        // }

        // RCLCPP_WARN_STREAM(get_logger(), "---");

        // IkReal eetrans[3];
        // auto trans_vec = Eigen::Vector3f(ee_aff.translation());
        // for (size_t i = 0; i < 3; ++i)
        //     eetrans[i] = trans_vec(i);
        // eetrans[0] -= 0.18;
        // eetrans[1] += 0.35;

        // for (size_t i = 0; i < 3; ++i)
        // {
        //     RCLCPP_WARN_STREAM(get_logger(), eetrans[i]);
        // }

        // // for (std::size_t i = 0; i < vfree.size(); ++i)
        // //     vfree[i] = atof(argv[13 + i]);
        // IkSolutionList<IkReal> solutions;
        // bool success = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

        // if (!success)
        // {
        //     RCLCPP_ERROR(get_logger(), "Cannot calculate IK. Aborting...");
        //     // _generated_path_pub->publish(custom_interfaces::msg::GeneratedPath());
        //     // goal_handle->abort(result);
        //     // return;
        // }
        // else
        // {
        //     printf("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
        //     std::vector<IkReal> solvalues(GetNumJoints());
        //     for (std::size_t i = 0; i < solutions.GetNumSolutions(); ++i)
        //     {
        //         const IkSolutionBase<IkReal> &sol = solutions.GetSolution(i);
        //         printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
        //         std::vector<IkReal> vsolfree(sol.GetFree().size());
        //         sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);
        //         for (std::size_t j = 0; j < solvalues.size(); ++j)
        //             printf("%.15f, ", solvalues[j]);

        //         _setJointStates(solvalues);

        //         cv::Mat img = cv::Mat::ones(100, 100, CV_8UC1) * 255;

        //         cv::putText(img,                         //target image
        //                     std::to_string(i),            //text
        //                     cv::Point(10, img.rows / 2), //top-left position
        //                     cv::FONT_HERSHEY_DUPLEX,
        //                     1.0,
        //                     CV_RGB(118, 185, 0), //font color
        //                     2);
        //         cv::imshow("kek", img);
        //         cv::waitKey();
        //         printf("\n");
        //     }
        // }

        // Check goal state validity
        _setJointStates(path_planning_input.goal_state);

        cv::imshow("kek", cv::Mat::ones(100, 100, CV_8UC1) * 255);
        cv::waitKey();

        if (_validateJointStates(path_planning_input.goal_state, _robot_info.limits) != ReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Invalid final state. Joint states are outside of limits. Aborting...");
            _generated_path_pub->publish(custom_interfaces::msg::GeneratedPath());
            goal_handle->abort(result);
            return;
        }

        if (_validateEndEffectorPose(end_effector_pose, 0.01) != ReturnCode::SUCCESS)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Invalid final state. End effector is in invalid position. Aborting...");
            _generated_path_pub->publish(custom_interfaces::msg::GeneratedPath());
            goal_handle->abort(result);
            return;
        }

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

    ReturnCode GeneratePath::_validateJointStates(const ArmConfiguration &joint_states, const std::vector<Limits> &joint_limits)
    {
        if (joint_states.size() != joint_limits.size())
        {
            RCLCPP_WARN_STREAM(get_logger(), "Amount of joint states (" << joint_states.size() << ") is different than amount of joint limits (" << joint_limits.size() << ")");
            return ReturnCode::FAILURE;
        }
        for (auto i = 0; i < joint_states.size(); ++i)
            if (joint_states[i] < joint_limits[i].lower || joint_states[i] > joint_limits[i].upper)
                return ReturnCode::FAILURE;
        return ReturnCode::SUCCESS;
    }

    ReturnCode GeneratePath::_validateEndEffectorPose(const Eigen::Affine3d &goal_end_effector_pose, const double &error_threshold)
    {
        b3LinkState link_state;
        if (!_scene_info->bullet_client->getLinkState(_scene_info->robot_idx, _scene_info->end_effector_idx, 0, 0, &link_state))
            return ReturnCode::FAILURE;

        Eigen::Vector3d ee_position(link_state.m_worldLinkFramePosition[0], link_state.m_worldLinkFramePosition[1], link_state.m_worldLinkFramePosition[2]);
        Eigen::Vector3d goal_ee_position(goal_end_effector_pose.translation());
        auto ee_position_dist = (ee_position - goal_ee_position).norm();
        return ee_position_dist < error_threshold ? ReturnCode::SUCCESS : ReturnCode::FAILURE;
    }

    ArmConfiguration GeneratePath::_calculateGoalStateFromEndEffectorPose(const Eigen::Affine3d &end_effector_pose)
    {
        b3RobotSimulatorInverseKinematicArgs ik_args;
        ik_args.m_bodyUniqueId = _scene_info->robot_idx;
        ik_args.m_endEffectorLinkIndex = _scene_info->end_effector_idx;
        Eigen::Vector3d end_effector_position(end_effector_pose.translation());
        Eigen::Quaterniond end_effector_orientation(end_effector_pose.rotation());
        ik_args.m_endEffectorTargetPosition[0] = end_effector_position.x();
        ik_args.m_endEffectorTargetPosition[1] = end_effector_position.y();
        ik_args.m_endEffectorTargetPosition[2] = end_effector_position.z();
        ik_args.m_endEffectorTargetOrientation[0] = end_effector_orientation.x();
        ik_args.m_endEffectorTargetOrientation[1] = end_effector_orientation.y();
        ik_args.m_endEffectorTargetOrientation[2] = end_effector_orientation.z();
        ik_args.m_endEffectorTargetOrientation[3] = end_effector_orientation.w();
        ik_args.m_numDegreeOfFreedom = _robot_info.nr_joints;

        ik_args.m_lowerLimits.resize(_robot_info.limits.size());
        ik_args.m_upperLimits.resize(_robot_info.limits.size());
        ik_args.m_jointRanges.resize(_robot_info.limits.size());
        ik_args.m_restPoses.resize(_robot_info.limits.size());
        ik_args.m_jointDamping.resize(_robot_info.limits.size());
        for (size_t i = 0; i < _robot_info.limits.size(); ++i)
        {
            ik_args.m_lowerLimits[i] = _ik_joint_limits[i].lower;
            ik_args.m_upperLimits[i] = _ik_joint_limits[i].upper;
            ik_args.m_jointRanges[i] = std::abs(_ik_joint_limits[i].lower) + std::abs(_ik_joint_limits[i].upper);
            // RCLCPP_WARN_STREAM(get_logger(), ik_args.m_lowerLimits[i] << ", " << ik_args.m_upperLimits[i]);
            ik_args.m_restPoses[i] = 0;
            // ik_args.m_jointDamping[i] = 0.1;
        }
        // ik_args.m_jointDamping[5] = 1;

        ik_args.m_flags |= B3_HAS_IK_TARGET_ORIENTATION;
        ik_args.m_flags |= B3_HAS_NULL_SPACE_VELOCITY;

        b3RobotSimulatorInverseKinematicsResults ik_results;
        _scene_info->bullet_client->calculateIK(ik_args, ik_results);

        ArmConfiguration goal_configuration(_robot_info.nr_joints);
        for (size_t i = 0; i < goal_configuration.size(); i++)
            goal_configuration[i] = ik_results.m_calculatedJointPositions[i];

        //////////////////////////////////////////////////////////
        // End effector pose visualization
        // Eigen::Affine3f ee_aff;
        // auto ros_ee_aff = end_effector_pose;
        // helpers::converters::geometryToEigenAffine(ros_ee_aff, ee_aff);

        // Eigen::Vector3f ee_pos(ee_aff.translation());
        // Eigen::Quaternionf ee_quat(ee_aff.rotation());
        Eigen::Vector3d x_shift = end_effector_orientation.toRotationMatrix().col(0) * 0.2;
        Eigen::Vector3d ee_pos_x = end_effector_position + x_shift;

        Eigen::Vector3d y_shift = end_effector_orientation.toRotationMatrix().col(1) * 0.2;
        Eigen::Vector3d ee_pos_y = end_effector_position + y_shift;

        Eigen::Vector3d z_shift = end_effector_orientation.toRotationMatrix().col(2) * 0.2;
        Eigen::Vector3d ee_pos_z = end_effector_position + z_shift;

        b3RobotSimulatorAddUserDebugLineArgs LINE_ARGS;
        LINE_ARGS.m_colorRGB[0] = 1;
        LINE_ARGS.m_colorRGB[1] = 0;
        LINE_ARGS.m_colorRGB[2] = 0;
        btVector3 from(end_effector_position.x(), end_effector_position.y(), end_effector_position.z());
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

    void GeneratePath::_updateJointLimits()
    {
        const double range_tighten_coeff = 0.95; // values from 0.0 - 1.0 representing how much range should be tighten
        for (size_t i = 0; i < _robot_info.limits.size(); ++i)
        {
            auto range_middle = (_robot_info.limits[i].lower + _robot_info.limits[i].upper) / 2.0;
            auto range_middle_to_limit_dist = _robot_info.limits[i].upper - range_middle;
            auto offset = (range_middle_to_limit_dist * (1.0 - range_tighten_coeff)) / 2.0;
            _ik_joint_limits.push_back(Limits(_robot_info.limits[i].lower + offset * 2, _robot_info.limits[i].upper - offset * 2));
            _robot_info.limits[i].lower += offset;
            _robot_info.limits[i].upper -= offset;
        }

        std::stringstream ss;
        ss << std::endl
           << "Limits for path planning:" << std::endl;
        for (size_t i = 0; i < _robot_info.limits.size(); ++i)
            ss << "  joint " << i + 1 << ": (" << _robot_info.limits[i].lower << ", " << _robot_info.limits[i].upper << ")" << std::endl;

        ss << "Limits for IK:" << std::endl;
        for (size_t i = 0; i < _ik_joint_limits.size(); ++i)
            ss << "  joint " << i + 1 << ": (" << _ik_joint_limits[i].lower << ", " << _ik_joint_limits[i].upper << ")" << std::endl;
        RCLCPP_DEBUG(get_logger(), ss.str());
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
