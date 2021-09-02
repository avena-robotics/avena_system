#include "physics_client_handler/physics_client_handler.hpp"

namespace physics_client_handler
{
    PhysicsClientHandler::PhysicsClientHandler(const helpers::commons::RobotInfo &robot_info, const rclcpp::Logger &logger)
        : _robot_info(robot_info),
          _logger(logger)
    {
        // Physics server handler initialization
        _bullet_client = std::make_shared<bullet_client::b3RobotSimulatorClientAPI>();
        if (!_bullet_client->connect(eCONNECT_SHARED_MEMORY))
            throw PhysicsClientHandlerError("Cannot connect to Bullet physics server");
        _bullet_client->syncBodies();
        _readSceneInfoFromPhysicsServer(_robot_info);
        _num_degrees_of_freedom = _robot_info.nr_joints;
    }

    PhysicsClientHandler::~PhysicsClientHandler()
    {
        if (_bullet_client->isConnected())
            _bullet_client->disconnect();
    }

    ArmConfiguration PhysicsClientHandler::computeIk(const Eigen::Affine3d &end_effector_pose,
                                                     const std::vector<Limits> &joint_limits,
                                                     const Obstacles &collision_objects)
    {
        auto initial_state = getJointStates();

        const size_t max_iter = 10;

        Eigen::Vector3d ee_position(end_effector_pose.translation());
        Eigen::Quaterniond ee_orientation(end_effector_pose.rotation());

        b3RobotSimulatorInverseKinematicArgs args;
        args.m_bodyUniqueId = _robot_idx;
        args.m_endEffectorLinkIndex = _end_effector_idx;
        args.m_numDegreeOfFreedom = _num_degrees_of_freedom;
        args.m_endEffectorTargetPosition[0] = ee_position.x();
        args.m_endEffectorTargetPosition[1] = ee_position.y();
        args.m_endEffectorTargetPosition[2] = ee_position.z();
        args.m_endEffectorTargetOrientation[0] = ee_orientation.x();
        args.m_endEffectorTargetOrientation[1] = ee_orientation.y();
        args.m_endEffectorTargetOrientation[2] = ee_orientation.z();
        args.m_endEffectorTargetOrientation[3] = ee_orientation.w();
        args.m_flags |= B3_HAS_IK_TARGET_ORIENTATION;

        ArmConfiguration goal_configuration(_num_degrees_of_freedom);
        bool found_solution = false;

        auto start_ik = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_ik < std::chrono::milliseconds(500))
        {
            setJointStates(initial_state);

            bool close_enough = false;
            size_t iter = 0;

            while (!close_enough && iter++ < max_iter)
            {
                b3RobotSimulatorInverseKinematicsResults result;
                if (_bullet_client->calculateIK(args, result))
                {
                    for (size_t i = 0; i < _num_degrees_of_freedom; ++i)
                        goal_configuration[i] = result.m_calculatedJointPositions[i];
                    // RCLCPP_INFO_STREAM(_logger, "[IK]: size: " << goal_configuration.size());

                    setJointStates(goal_configuration);
                    b3LinkState ee_state;
                    _bullet_client->getLinkState(_robot_idx, _end_effector_idx, 0, 0, &ee_state);
                    Eigen::Translation3d ee_calculated_position(ee_state.m_worldLinkFramePosition[0], ee_state.m_worldLinkFramePosition[1], ee_state.m_worldLinkFramePosition[2]);
                    Eigen::Quaterniond ee_calculated_orientation(ee_state.m_worldLinkFrameOrientation[3], ee_state.m_worldLinkFrameOrientation[0], ee_state.m_worldLinkFrameOrientation[1], ee_state.m_worldLinkFrameOrientation[2]);
                    Eigen::Affine3d calculated_end_effector_pose = ee_calculated_position * ee_calculated_orientation;

                    auto aff_diff = helpers::commons::getDiffBetweenAffines(end_effector_pose, calculated_end_effector_pose);
                    close_enough = (aff_diff[0] < POSITION_THRESHOLD) &&
                                   std::abs(aff_diff[1]) < ORIENTATION_THRESHOLD &&
                                   std::abs(aff_diff[2]) < ORIENTATION_THRESHOLD &&
                                   std::abs(aff_diff[3]) < ORIENTATION_THRESHOLD;
                }
                else
                    RCLCPP_DEBUG(_logger, "[IK]: Bullet cannot solve IK problem");
            }

            if (close_enough)
            {
                bool in_limits = true;
                for (size_t i = 0; i < _num_degrees_of_freedom; ++i)
                {
                    if (goal_configuration[i] < joint_limits[i].lower || goal_configuration[i] > joint_limits[i].upper)
                    {
                        RCLCPP_DEBUG_STREAM(_logger, "[IK]: Joint " << i + 1 << " out of limits. Value: " << goal_configuration[i]
                                                                    << " [rad]. Limits: [" << joint_limits[i].lower << ", "
                                                                    << joint_limits[i].upper << " [rad] [rad]");
                        in_limits = false;
                        break;
                    }
                }

                if (in_limits && !inCollision(collision_objects))
                {
                    found_solution = true;
                    break;
                }
            }
            else
                RCLCPP_DEBUG(_logger, "[IK]: Found solution is not close enough");

            std::stringstream ss;
            ss << "Initial joint states for numerical IK solver: ";
            for (size_t i = 0; i < _num_degrees_of_freedom; ++i)
            {
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_real_distribution<> distr(joint_limits[i].lower, joint_limits[i].upper);
                initial_state[i] = distr(gen);
                ss << initial_state[i];
                if (i < _num_degrees_of_freedom - 1)
                    ss << ", ";
            }
            RCLCPP_DEBUG(_logger, "[IK]: " + ss.str());
        }
        auto stop_ik = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration<double, std::milli>(stop_ik - start_ik);
        RCLCPP_DEBUG_STREAM(_logger, "[IK]: Bullet inverse kinematics took: " << duration.count() << " [ms]");

        return found_solution ? goal_configuration : ArmConfiguration();
    }

    bool PhysicsClientHandler::inCollision(const Obstacles &obstacles)
    {
        int contacts_amount = 0;
        // Check collision with obstacles
        for (auto obstacle_idx : obstacles)
        {
            b3RobotSimulatorGetContactPointsArgs obtacles_collision_args;
            obtacles_collision_args.m_bodyUniqueIdA = _robot_idx;
            obtacles_collision_args.m_bodyUniqueIdB = obstacle_idx;
            b3ContactInformation contact_info;
            _bullet_client->getClosestPoints(obtacles_collision_args, SAFETY_DISTANCE, &contact_info);
            contacts_amount += contact_info.m_numContactPoints;
        }

        // Check self collision
        // TODO: Handle links without any collision shape e.g. "prefix_gripper_connection"
        b3ContactInformation contact_info;
        b3RobotSimulatorGetContactPointsArgs self_collision_args;
        self_collision_args.m_bodyUniqueIdA = _robot_idx;
        self_collision_args.m_bodyUniqueIdB = _robot_idx;
        for (int i = 0; i < _bullet_client->getNumJoints(_robot_idx) - 2; i++)
        {
            self_collision_args.m_linkIndexA = i;
            for (int j = i + 2; j < _bullet_client->getNumJoints(_robot_idx); j++)
            {
                self_collision_args.m_linkIndexB = j;
                _bullet_client->getClosestPoints(self_collision_args, SAFETY_DISTANCE, &contact_info);
                contacts_amount += contact_info.m_numContactPoints;
            }
        }
        return contacts_amount > CONTACT_NUMBER_ALLOWED;
    }

    bool PhysicsClientHandler::inCollision()
    {
        auto &obstacles = getCollisionObjectsHandles();
        return inCollision(obstacles);
    }

    const Obstacles &PhysicsClientHandler::getCollisionObjectsHandles()
    {
        _bullet_client->syncBodies();
        _obstacles.clear();
        int num_bodies = _bullet_client->getNumBodies();
        for (int body_id = 0; body_id < num_bodies; ++body_id)
        {
            int body_unique_id = _bullet_client->getBodyUniqueId(body_id);
            b3BodyInfo body_info;
            _bullet_client->getBodyInfo(body_unique_id, &body_info);
            if (std::strcmp(body_info.m_bodyName, _robot_info.robot_name.c_str()) != 0)
                _obstacles.push_back(body_unique_id);
        }
        return _obstacles;
    }

    void PhysicsClientHandler::setJointStates(const ArmConfiguration &joint_states)
    {
        if (joint_states.size() != _joint_handles.size())
            throw PhysicsClientHandlerError("Number of joint states to be set is invalid. Input joint state size: " +
                                            std::to_string(joint_states.size()) + " and it should be equal to " + std::to_string(_joint_handles.size()));

        for (size_t i = 0; i < _joint_handles.size(); i++)
            _bullet_client->resetJointState(_robot_idx, _joint_handles[i], joint_states[i]);
    }

    ArmConfiguration PhysicsClientHandler::getJointStates()
    {
        b3JointStates2 server_joint_states;
        _bullet_client->getJointStates(_robot_idx, server_joint_states);
        ArmConfiguration joint_states(_q_indices.size());
        for (size_t i = 0; i < _q_indices.size(); ++i)
            joint_states[i] = server_joint_states.m_actualStateQ[_q_indices[i]];

        // std::cout << "joint states" << std::endl;
        // for (size_t i = 0; i < joint_states.size(); ++i)
        //     std::cout << joint_states[i] << std::endl;

        return joint_states;
    }

    void PhysicsClientHandler::_readSceneInfoFromPhysicsServer(const helpers::commons::RobotInfo &robot_info)
    {
        _bullet_client->syncBodies();
        int num_bodies = _bullet_client->getNumBodies();
        if (num_bodies == 0)
            throw PhysicsClientHandlerError("There are no objects in the physics server");

        _joint_handles.clear();
        _q_indices.clear();
        for (int body_id = 0; body_id < num_bodies; ++body_id)
        {
            int body_unique_id = _bullet_client->getBodyUniqueId(body_id);
            b3BodyInfo body_info;
            _bullet_client->getBodyInfo(body_unique_id, &body_info);
            if (std::strcmp(body_info.m_bodyName, robot_info.robot_name.c_str()) == 0)
            {
                _robot_idx = body_unique_id;
                int num_joints = _bullet_client->getNumJoints(body_unique_id);
                for (int joint_idx = 0; joint_idx < num_joints; ++joint_idx)
                {
                    b3JointInfo joint_info;
                    _bullet_client->getJointInfo(body_unique_id, joint_idx, &joint_info);
                    if (std::strcmp(joint_info.m_linkName, robot_info.connection.c_str()) == 0)
                        _end_effector_idx = joint_idx;

                    if (joint_info.m_jointType != JointType::eFixedType)
                    {
                        auto joint_name_it = std::find(robot_info.joint_names.cbegin(), robot_info.joint_names.cend(), std::string(joint_info.m_jointName));
                        if (joint_name_it != robot_info.joint_names.end())
                        {
                            _q_indices.push_back(joint_info.m_qIndex);
                            _joint_handles.push_back(joint_idx);
                        }
                    }
                }
            }
        }

        if (_end_effector_idx == INVALID_HANDLE)
            throw PhysicsClientHandlerError("End effector handle was not read from the physics server correctly");

        if (_robot_idx == INVALID_HANDLE)
            throw PhysicsClientHandlerError("Robot handle was not read from the physics server correctly");

        if (_joint_handles.size() != robot_info.nr_joints)
        {
            std::stringstream ss;
            ss << "Number of robot joints read from the physics server \
                   is not equal to number of joints in robot description (physics server: "
               << _joint_handles.size()
               << ", robot description: " << robot_info.nr_joints << ")";
            throw PhysicsClientHandlerError(ss.str());
        }
    }

    std::optional<Eigen::Affine3d> PhysicsClientHandler::getEndEffectorPose()
    {
        _bullet_client->syncBodies();
        b3LinkState link_state;
        if (!_bullet_client->getLinkState(_robot_idx, _end_effector_idx, 0, 0, &link_state))
            return std::nullopt;

        Eigen::Translation3d ee_position(link_state.m_worldLinkFramePosition[0], link_state.m_worldLinkFramePosition[1], link_state.m_worldLinkFramePosition[2]);
        Eigen::Quaterniond ee_orientation(link_state.m_worldLinkFrameOrientation[3], link_state.m_worldLinkFrameOrientation[0], link_state.m_worldLinkFrameOrientation[1], link_state.m_worldLinkFrameOrientation[2]);
        return ee_position * ee_orientation;
    }

    void PhysicsClientHandler::drawCoordinateAxes(const Eigen::Affine3d &pose)
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
        _bullet_client->addUserDebugLine(from, to_x, LINE_ARGS);
        LINE_ARGS.m_colorRGB[0] = 0;
        LINE_ARGS.m_colorRGB[1] = 1;
        LINE_ARGS.m_colorRGB[2] = 0;
        btVector3 to_y(pos_y.x(), pos_y.y(), pos_y.z());
        _bullet_client->addUserDebugLine(from, to_y, LINE_ARGS);
        LINE_ARGS.m_colorRGB[0] = 0;
        LINE_ARGS.m_colorRGB[1] = 0;
        LINE_ARGS.m_colorRGB[2] = 1;
        btVector3 to_z(pos_z.x(), pos_z.y(), pos_z.z());
        _bullet_client->addUserDebugLine(from, to_z, LINE_ARGS);
    }

    bool PhysicsClientHandler::isSceneValid()
    {
        int num_joints = _bullet_client->getNumJoints(_robot_idx);
        size_t num_moving_joints = 0;
        for (int joint_idx = 0; joint_idx < num_joints; ++joint_idx)
        {
            b3JointInfo joint_info;
            _bullet_client->getJointInfo(_robot_idx, joint_idx, &joint_info);
            if (joint_info.m_jointType != JointType::eFixedType)
            {
                auto joint_name_it = std::find(_robot_info.joint_names.begin(), _robot_info.joint_names.end(), std::string(joint_info.m_jointName));
                if (joint_name_it != _robot_info.joint_names.end())
                    num_moving_joints++;
            }
        }
        return num_moving_joints == _robot_info.nr_joints;
    }

    void PhysicsClientHandler::cleanDebugItems()
    {
        _bullet_client->removeAllUserDebugItems();
    }

    void PhysicsClientHandler::syncWithPhysicsServer()
    {
        _bullet_client->syncBodies();
    }

    void PhysicsClientHandler::initializeConnection()
    {
        syncWithPhysicsServer();
        cleanDebugItems();
    }

} // namespace physics_client_handler
