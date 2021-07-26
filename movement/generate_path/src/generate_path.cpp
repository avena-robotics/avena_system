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

        _bullet_client = std::make_shared<bullet_client::b3RobotSimulatorClientAPI>();
        bool connected = _bullet_client->connect(eCONNECT_SHARED_MEMORY);
        if (!connected)
        {
            RCLCPP_WARN(get_logger(), "Cannot connect to physics server");
            return ReturnCode::FAILURE;
        }
        _bullet_client->syncBodies();
        RCLCPP_INFO_STREAM(get_logger(), "Connected to physics server successfully. API version: " << _bullet_client->getAPIVersion());

        return ReturnCode::SUCCESS;
    }

    ReturnCode GeneratePath::_shutdown()
    {
        _joint_state_sub.reset();
        _generated_path_pub.reset();
        _action_server_pose.reset();
        if (_bullet_client->isConnected())
            _bullet_client->disconnect();
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
        sensor_msgs::msg::JointState::SharedPtr current_joint_states;
        {
            std::lock_guard<std::mutex> lg(_current_joint_states_mtx);
            current_joint_states = std::make_shared<sensor_msgs::msg::JointState>(*_current_joint_states);
        }

        _bullet_client->removeAllUserDebugItems();

        RCLCPP_INFO(get_logger(), "Generating pose path");
        _bullet_client->syncBodies();
        auto result = std::make_shared<GeneratePathPose::Result>();

        _setCurrentJointStatesOnPhysicsServer(current_joint_states);
        
        const float safety_range = 0.003;
        const int contact_number_allowed = 1;

        // Check initial state
        if (_calculateContactPointsAmount(safety_range) > contact_number_allowed)
        {
            RCLCPP_ERROR(get_logger(), "Invalid initial state. Robot is in collision with scene or itself. Aborting...");
            _generated_path_pub->publish(custom_interfaces::msg::GeneratedPath());
            goal_handle->abort(result);
            return;
        }

        // ///////////////////////////////////////////////////////////////////////////
        // cv::imshow("kek", cv::Mat::ones(100, 100, CV_8UC1) * 128);
        // cv::waitKey();
        // ///////////////////////////////////////////////////////////////////////////

        auto end_effector_pose = goal_handle.get()->get_goal()->end_effector_pose;
        auto goal_state = _calculateGoalStateFromEndEffectorPose(end_effector_pose, current_joint_states);

        // Set arm to goal state and check collisions
        for (int i = 0; i < END_EFECTOR_LINK_INDEX; i++)
        {
            _bullet_client->resetJointState(_robot_idx, i, goal_state[i]);
        }

        if (_calculateContactPointsAmount(safety_range) > contact_number_allowed)
        {
            RCLCPP_ERROR(get_logger(), "Invalid final state. Robot is in collision with scene or itself. Aborting...");
            _generated_path_pub->publish(custom_interfaces::msg::GeneratedPath());
            goal_handle->abort(result);
            return;
        }

        _setCurrentJointStatesOnPhysicsServer(current_joint_states);

        {
            // TODO: Remove later
            std::stringstream ss;
            ss << "Goal arm config: ";
            for (auto ac : goal_state)
            {
                ss << ac << ", ";
            }
            RCLCPP_INFO(get_logger(), ss.str());
        }

        // cv::imshow("kek", cv::Mat::ones(100, 100, CV_8UC1) * 255);
        // cv::waitKey();

        std::vector<float> joint_states(current_joint_states->position.begin(), current_joint_states->position.end());
        std::vector<int> obstacles = {_table_idx};
        std::vector<int> robot = {_robot_idx};
        int max_iter = 10000;
        float delta_q = 0.15;
        float steer_goal_p = 0.1;
        std::vector<ArmConfiguration> path;
        {
            helpers::Timer timer("RRT algorithm", get_logger());
            path = rrt(joint_states, goal_state, max_iter, delta_q, steer_goal_p, _bullet_client.get(), safety_range, obstacles, robot, contact_number_allowed);
        }
        RCLCPP_INFO_STREAM(get_logger(), "Length of generated path: " << path.size());

        if (path.size() == 0)
        {
            RCLCPP_ERROR(get_logger(), "Planner was not able to generate path. Aborting...");
            _generated_path_pub->publish(custom_interfaces::msg::GeneratedPath());
            goal_handle->abort(result);
            return;
        }

        // Set arm to last config for visualization
        {
            auto last_arm_config = path.back();
            // std::stringstream ss;
            // ss << "Last config: ";
            // for (auto ac : last_arm_config)
            // {
            //     ss << ac << ", ";
            // }
            // RCLCPP_WARN(get_logger(), ss.str());
            for (int i = 0; i < END_EFECTOR_LINK_INDEX; i++)
            {
                _bullet_client->resetJointState(_robot_idx, i, last_arm_config[i]);
            }
        }

        // //////////////////////////////////////////////////////////////////////
        // // Set for every configuration in result and check collisions because there is something wrong
        // for (auto arm_config : path)
        // {
        //     for (int i = 0; i < END_EFECTOR_LINK_INDEX; i++)
        //     {
        //         _bullet_client->resetJointState(_robot_idx, i, arm_config[i]);
        //     }
        //     RCLCPP_ERROR_STREAM(get_logger(), "Contacts amount: " << _calculateContactPointsAmount(safety_range));
        //     cv::imshow("kek", cv::Mat::ones(100, 100, CV_8UC1) * 255);
        //     cv::waitKey();
        // }
        // //////////////////////////////////////////////////////////////////////

        custom_interfaces::msg::GeneratedPath generated_path;
        generated_path.header.stamp = now();
        generated_path.path_segments.resize(1);
        _convertPathSegmentToTrajectoryMsg(path, generated_path.path_segments[0]);

        _generated_path_pub->publish(generated_path);
        if (rclcpp::ok())
        {
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal to pose succeeded");
        }
        RCLCPP_INFO(get_logger(), "Generate to pose finished");
    }

    int GeneratePath::_calculateContactPointsAmount(const float &distance)
    {
        int contacts_amount = 0;
        b3RobotSimulatorGetContactPointsArgs collision_args;
        collision_args.m_bodyUniqueIdA = _robot_idx;
        collision_args.m_bodyUniqueIdB = _table_idx;
        b3ContactInformation contact_info;
        _bullet_client->getClosestPoints(collision_args, distance, &contact_info);
        // _bullet_client->getContactPoints(collision_args, &contact_info);
        contacts_amount += contact_info.m_numContactPoints;

        // Check self collision
        collision_args.m_bodyUniqueIdA = _robot_idx;
        collision_args.m_bodyUniqueIdB = _robot_idx;
        for (int i = 0; i < _bullet_client->getNumJoints(_robot_idx) - 2; i++)
        {
            // b3JointInfo joint_info;
            // _bullet_client->getJointInfo(_robot_idx, i, &joint_info);
            // std::cout << joint_info.m_jointName << ", " << joint_info.m_linkName << std::endl;
            collision_args.m_linkIndexA = i;
            for (int j = i + 2; j < _bullet_client->getNumJoints(_robot_idx); j++)
            {
                // b3JointInfo joint_info_inner;
                // _bullet_client->getJointInfo(_robot_idx, j, &joint_info_inner);
                // std::cout << "\t" << joint_info_inner.m_jointName << ", " << joint_info_inner.m_linkName << std::endl;
                collision_args.m_linkIndexB = j;
                _bullet_client->getClosestPoints(collision_args, distance, &contact_info);
                contacts_amount += contact_info.m_numContactPoints;
            }
        }
        return contacts_amount;
    }

    ArmConfiguration GeneratePath::_calculateGoalStateFromEndEffectorPose(const geometry_msgs::msg::Pose &end_effector_pose, 
                                                                          const sensor_msgs::msg::JointState::SharedPtr &/*current_joint_states*/)
    {
        b3RobotSimulatorInverseKinematicArgs ik_args;
        ik_args.m_bodyUniqueId = _robot_idx;
        ik_args.m_endEffectorLinkIndex = _end_effector_idx;
        ik_args.m_endEffectorTargetPosition[0] = end_effector_pose.position.x;
        ik_args.m_endEffectorTargetPosition[1] = end_effector_pose.position.y;
        ik_args.m_endEffectorTargetPosition[2] = end_effector_pose.position.z + 0.545; // FIXME: Change the scene
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
            // ik_args.m_jointDamping[i] = 0.1;
        }
        // ik_args.m_jointDamping[5] = 1;
        // ik_args.m_restPoses[3] = -0.9;
        // ik_args.m_restPoses[5] = 2.47;

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
        ros_ee_aff.position.z += 0.545;
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
        _bullet_client->addUserDebugLine(from, to_x, LINE_ARGS);
        LINE_ARGS.m_colorRGB[0] = 0;
        LINE_ARGS.m_colorRGB[1] = 1;
        LINE_ARGS.m_colorRGB[2] = 0;
        btVector3 to_y(ee_pos_y.x(), ee_pos_y.y(), ee_pos_y.z());
        _bullet_client->addUserDebugLine(from, to_y, LINE_ARGS);
        LINE_ARGS.m_colorRGB[0] = 0;
        LINE_ARGS.m_colorRGB[1] = 0;
        LINE_ARGS.m_colorRGB[2] = 1;
        btVector3 to_z(ee_pos_z.x(), ee_pos_z.y(), ee_pos_z.z());
        _bullet_client->addUserDebugLine(from, to_z, LINE_ARGS);
        //////////////////////////////////////////////////////////

        b3RobotSimulatorInverseKinematicsResults ik_results;
        // _bullet_client->calculateInverseKinematics(ik_args, ik_results);
        _bullet_client->calculateIK(ik_args, ik_results);

        // for (int i = 0; i < ik_results.m_calculatedJointPositions.size(); i++)
        // {
        //     std::cout << "i: " << i << ", " << ik_results.m_calculatedJointPositions[i] << std::endl;
        // }

        ArmConfiguration goal_configuration(_robot_info.nr_joints);
        for (size_t i = 0; i < goal_configuration.size(); i++)
            goal_configuration[i] = ik_results.m_calculatedJointPositions[i];

        return goal_configuration;
    }

    ReturnCode GeneratePath::_setCurrentJointStatesOnPhysicsServer(const sensor_msgs::msg::JointState::SharedPtr &joint_states)
    {
        int num_bodies = _bullet_client->getNumBodies();
        for (int body_id = 0; body_id < num_bodies; ++body_id)
        {
            int body_unique_id = _bullet_client->getBodyUniqueId(body_id);
            b3BodyInfo body_info;
            _bullet_client->getBodyInfo(body_unique_id, &body_info);

            // std::cout << "m_baseName: " << body_info.m_baseName << ", m_bodyName: " << body_info.m_bodyName << std::endl;

            if (std::strcmp(body_info.m_bodyName, _robot_info.robot_name.c_str()) == 0)
            {
                _robot_idx = body_unique_id;
                // _temp_robot_idx = body_id;
                // std::cout << "body id: " << body_id << std::endl;
                // std::cout << "body unique id: " << body_unique_id << std::endl;
                // std::cout << "m_baseName: " << body_info.m_baseName << ", m_bodyName: " << body_info.m_bodyName << std::endl;

                int num_joints = _bullet_client->getNumJoints(body_unique_id);
                // std::cout << "num_joints: " << num_joints << std::endl;
                for (int joint_idx = 0; joint_idx < num_joints; ++joint_idx)
                {
                    b3JointInfo joint_info;
                    _bullet_client->getJointInfo(body_unique_id, joint_idx, &joint_info);

                    // RCLCPP_WARN_STREAM(get_logger(), joint_info.m_jointName << ", " << joint_info.m_linkName);

                    if (std::strcmp(joint_info.m_linkName, _robot_info.connection.c_str()) == 0)
                        _end_effector_idx = joint_idx;

                    if (joint_info.m_jointType != JointType::eFixedType)
                    {
                        auto joint_name_it = std::find(joint_states->name.begin(), joint_states->name.end(), std::string(joint_info.m_jointName));
                        auto joint_position_idx = std::distance(joint_states->name.begin(), joint_name_it);
                        float joint_val = joint_states->position[joint_position_idx];
                        _bullet_client->resetJointState(body_unique_id, joint_idx, joint_val);
                    }
                }
            }
            else
            {
                // _temp_table_idx = body_id;
                _table_idx = body_unique_id;
            }
        }

        RCLCPP_INFO_STREAM(get_logger(), "End effector ID: " << _end_effector_idx);

        return ReturnCode::SUCCESS;
    }

    // ReturnCode GeneratePath::_readSceneInfo()
    // {

    //     return ReturnCode::FAILURE;
    // }

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
