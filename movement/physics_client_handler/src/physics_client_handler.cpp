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

    std::optional<Eigen::MatrixXd> PhysicsClientHandler::getJacobian(const ArmConfiguration &joint_states)
    {
        std::vector<double> local_position(joint_states.size(), 0);
        std::vector<double> joint_velocities(joint_states.size(), 0);
        std::vector<double> joint_accelerations(joint_states.size(), 0);

        std::vector<double> linear_jacobian(joint_states.size() * 3);
        std::vector<double> angular_jacobian(joint_states.size() * 3);

        bool success = _bullet_client->getBodyJacobian(_robot_idx, _end_effector_idx, local_position.data(),
                                                       joint_states.data(), joint_velocities.data(), joint_accelerations.data(),
                                                       linear_jacobian.data(), angular_jacobian.data());
        if (!success)
            return std::nullopt;

        Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, joint_states.size());
        for (size_t i = 0; i < joint_states.size(); i++)
        {
            jacobian(0, i) = linear_jacobian[0 * joint_states.size() + i];
            jacobian(1, i) = linear_jacobian[1 * joint_states.size() + i];
            jacobian(2, i) = linear_jacobian[2 * joint_states.size() + i];

            jacobian(3, i) = angular_jacobian[0 * joint_states.size() + i];
            jacobian(4, i) = angular_jacobian[1 * joint_states.size() + i];
            jacobian(5, i) = angular_jacobian[2 * joint_states.size() + i];
        }

        return jacobian;
    }

    ArmConfiguration PhysicsClientHandler::computeIk(const Eigen::Affine3d &end_effector_pose,
                                                     const std::vector<Limits> &joint_limits,
                                                     const Obstacles &collision_objects,
                                                     const std::chrono::duration<double> &timeout)
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
        while (std::chrono::steady_clock::now() - start_ik < timeout)
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

    std::vector<ArmConfiguration> PhysicsClientHandler::computeAllIk(const Eigen::Affine3d &end_effector_pose,
                                                                     const std::vector<Limits> &joint_limits,
                                                                     const Obstacles &collision_objects,
                                                                     const std::chrono::duration<double> &timeout)
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

        std::vector<ArmConfiguration> goal_configurations;

        auto start_ik = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_ik < timeout)
        {
            ArmConfiguration single_goal_configuration(_num_degrees_of_freedom);

            setJointStates(initial_state);

            bool close_enough = false;
            size_t iter = 0;

            while (!close_enough && iter++ < max_iter && std::chrono::steady_clock::now() - start_ik < timeout)
            {
                b3RobotSimulatorInverseKinematicsResults result;
                if (_bullet_client->calculateIK(args, result))
                {
                    for (size_t i = 0; i < _num_degrees_of_freedom; ++i)
                        single_goal_configuration[i] = result.m_calculatedJointPositions[i];
                    // RCLCPP_INFO_STREAM(_logger, "[IK]: size: " << single_goal_configuration.size());

                    setJointStates(single_goal_configuration);
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
                    if (single_goal_configuration[i] < joint_limits[i].lower || single_goal_configuration[i] > joint_limits[i].upper)
                    {
                        RCLCPP_DEBUG_STREAM(_logger, "[IK]: Joint " << i + 1 << " out of limits. Value: " << single_goal_configuration[i]
                                                                    << " [rad]. Limits: [" << joint_limits[i].lower << ", "
                                                                    << joint_limits[i].upper << " [rad] [rad]");
                        in_limits = false;
                        break;
                    }
                }

                if (in_limits && !inCollision(collision_objects))
                {
                    goal_configurations.emplace_back(single_goal_configuration);
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
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop_ik - start_ik);
        RCLCPP_DEBUG_STREAM(_logger, "[IK]: Bullet inverse kinematics took: " << duration.count() << " [ms]");

        return goal_configurations;
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
        // RCLCPP_DEBUG_STREAM(_logger, "Number of collision points between robot and scene: " << contacts_amount);

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
                // if (contact_info.m_numContactPoints > 0)
                // {
                //     b3JointInfo link_a;
                //     _bullet_client->getJointInfo(_robot_idx, i, &link_a);
                //     b3JointInfo link_b;
                //     _bullet_client->getJointInfo(_robot_idx, j, &link_b);
                //     RCLCPP_DEBUG_STREAM(_logger, "Collision points: " << contact_info.m_numContactPoints
                //                                      << ", A: " << link_a.m_linkName << ", B: " << link_b.m_linkName);
                // }
            }
        }
        // RCLCPP_DEBUG_STREAM(_logger, "Number of collision points: " << contacts_amount);
        return contacts_amount > CONTACT_NUMBER_ALLOWED;
    }

    bool PhysicsClientHandler::inCollision()
    {
        auto &obstacles = getCollisionObjectsHandles();
        return inCollision(obstacles);
    }

    const Obstacles &PhysicsClientHandler::getCollisionObjectsHandles()
    {
        // _bullet_client->syncBodies();
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
        // _bullet_client->syncBodies();
        b3LinkState link_state;
        if (!_bullet_client->getLinkState(_robot_idx, _end_effector_idx, 0, 0, &link_state))
            return std::nullopt;

        Eigen::Translation3d ee_position(link_state.m_worldLinkFramePosition[0], link_state.m_worldLinkFramePosition[1], link_state.m_worldLinkFramePosition[2]);
        Eigen::Quaterniond ee_orientation(link_state.m_worldLinkFrameOrientation[3], link_state.m_worldLinkFrameOrientation[0], link_state.m_worldLinkFrameOrientation[1], link_state.m_worldLinkFrameOrientation[2]);
        return ee_position * ee_orientation;
    }

    int PhysicsClientHandler::createOctomap(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, const float grid_size)
    {
        if (point_cloud->size() == 0)
            return -2;

        std::vector<pcl::Vertices> triangles;
        _createMeshFromPointCloud(point_cloud, grid_size, triangles);

        int collision_mesh_body_uid = -1;
        {
            helpers::Timer timer("Spawning multibody mesh to physics server", _logger);

            // Extract vertices and indices from pcl::PolygonMesh to match with Bullet format
            std::vector<double> vertices; // flat sequence: pt_0_x, pt_0_y, pt_0_z, pt_1_x, pt_1_y, pt_1_z, pt_2_x, pt_2_y, pt_2_z, ...
            for (size_t pt_id = 0; pt_id < point_cloud->size(); pt_id++)
            {
                vertices.push_back(point_cloud->points[pt_id].x);
                vertices.push_back(point_cloud->points[pt_id].y);
                vertices.push_back(point_cloud->points[pt_id].z);
            }

            std::vector<int> indices;
            for (auto &polygon : triangles)
            {
                for (auto &vertex_id : polygon.vertices)
                {
                    indices.push_back(vertex_id);
                }
            }

            int collision_mesh_uid = _bullet_client->createCollisionShapeMesh(vertices, indices);

            b3RobotSimulatorCreateMultiBodyArgs args;
            args.m_baseMass = 0;
            args.m_basePosition = btVector3(0, 0, 0);
            args.m_baseOrientation = btQuaternion(0, 0, 0, 1);
            args.m_baseCollisionShapeIndex = collision_mesh_uid;
            collision_mesh_body_uid = _bullet_client->createMultiBody(args);
        }

        return collision_mesh_body_uid;
    }

    void PhysicsClientHandler::removeCollisions()
    {
        int num_bodies = _bullet_client->getNumBodies();
        if (num_bodies == 0)
            throw PhysicsClientHandlerError("There are no objects in the physics server");

        for (int body_id = 0; body_id < num_bodies; ++body_id)
        {
            int body_unique_id = _bullet_client->getBodyUniqueId(body_id);
            b3BodyInfo body_info;
            _bullet_client->getBodyInfo(body_unique_id, &body_info);
            if (!(std::strcmp(body_info.m_bodyName, _robot_info.robot_name.c_str()) == 0 || std::strcmp(body_info.m_bodyName, "table") == 0))
            {
                _bullet_client->removeBody(body_unique_id);
            }
        }
    }

    // void PhysicsClientHandler::drawCoordinateAxes(const Eigen::Affine3d &pose, double scale)
    // {
    //     Eigen::Vector3d position(pose.translation());
    //     Eigen::Quaterniond orientation(pose.rotation());
    // Eigen::Vector3d x_shift = orientation.toRotationMatrix().col(0) * scale;
    //     Eigen::Vector3d pos_x = position + x_shift;

    //     Eigen::Vector3d y_shift = orientation.toRotationMatrix().col(1) * scale;
    //     Eigen::Vector3d pos_y = position + y_shift;

    //     Eigen::Vector3d z_shift = orientation.toRotationMatrix().col(2) * scale;
    //     Eigen::Vector3d pos_z = position + z_shift;

    //     b3RobotSimulatorAddUserDebugLineArgs LINE_ARGS;
    //     LINE_ARGS.m_colorRGB[0] = 1;
    //     LINE_ARGS.m_colorRGB[1] = 0;
    //     LINE_ARGS.m_colorRGB[2] = 0;
    //     btVector3 from(position.x(), position.y(), position.z());
    //     btVector3 to_x(pos_x.x(), pos_x.y(), pos_x.z());
    //     _bullet_client->addUserDebugLine(from, to_x, LINE_ARGS);
    //     LINE_ARGS.m_colorRGB[0] = 0;
    //     LINE_ARGS.m_colorRGB[1] = 1;
    //     LINE_ARGS.m_colorRGB[2] = 0;
    //     btVector3 to_y(pos_y.x(), pos_y.y(), pos_y.z());
    //     _bullet_client->addUserDebugLine(from, to_y, LINE_ARGS);
    //     LINE_ARGS.m_colorRGB[0] = 0;
    //     LINE_ARGS.m_colorRGB[1] = 0;
    //     LINE_ARGS.m_colorRGB[2] = 1;
    //     btVector3 to_z(pos_z.x(), pos_z.y(), pos_z.z());
    //     _bullet_client->addUserDebugLine(from, to_z, LINE_ARGS);
    // }

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

    int PhysicsClientHandler::_createMeshFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, const float &grid_size, std::vector<pcl::Vertices> &out_triangles)
    {
        if (point_cloud->size() == 0)
            return 1;

        // RCLCPP_WARN_STREAM(_logger, "Amount of points before voxelization: " << point_cloud->size());
        {
            helpers::Timer timer("Input point cloud voxelization", _logger);
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(point_cloud);
            voxel_filter.setLeafSize(grid_size, grid_size, grid_size);
            voxel_filter.filter(*point_cloud);
        }
        // RCLCPP_WARN_STREAM(_logger, "Amount of points after voxelization: " << point_cloud->size());

        {
            helpers::Timer timer("Create mesh with PCL", _logger);
            // Normal estimation*
            pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(point_cloud);
            n.setInputCloud(point_cloud);
            n.setSearchMethod(tree);
            n.setKSearch(20);
            n.setNumberOfThreads(std::thread::hardware_concurrency());
            n.compute(*normals);
            //* normals should not contain the point normals + surface curvatures

            // Concatenate the XYZ and normal fields*
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
            pcl::concatenateFields(*point_cloud, *normals, *cloud_with_normals);
            //* cloud_with_normals = cloud + normals

            // Create search tree*
            pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
            tree2->setInputCloud(cloud_with_normals);

            // Initialize objects
            pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

            // Set the maximum distance between connected points (maximum edge length)
            gp3.setSearchRadius(grid_size * 2);

            // Set typical values for the parameters
            gp3.setMu(2.5);
            gp3.setMaximumNearestNeighbors(100);
            gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
            gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
            gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
            gp3.setNormalConsistency(false);

            // Get result
            gp3.setInputCloud(cloud_with_normals);
            gp3.setSearchMethod(tree2);
            gp3.reconstruct(out_triangles);
        }

        // {
        //     auto v0 = helpers::visualization::visualize({point_cloud}, {}, nullptr, "cloud");
        //     // Visualize
        //     pcl::PolygonMesh mesh;
        //     mesh.polygons = triangles;
        //     pcl::toPCLPointCloud2(*point_cloud, mesh.cloud);
        //     pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));
        //     viewer->setBackgroundColor(0, 0, 0);
        //     viewer->addPolygonMesh(mesh, "meshes", 0);
        //     viewer->addCoordinateSystem(1.0);
        //     viewer->initCameraParameters();
        //     viewer->spin();
        // }
        // auto viewer = helpers::visualization::visualize({pcl_octomap});

        return 0;
    }

    void PhysicsClientHandler::cleanDebugItems()
    {
        _bullet_client->removeAllUserDebugItems();
    }

    void PhysicsClientHandler::syncWithPhysicsServer()
    {
        _bullet_client->syncBodies();
    }

    void PhysicsClientHandler::refreshConnection()
    {
        syncWithPhysicsServer();
        cleanDebugItems();
    }

} // namespace physics_client_handler
