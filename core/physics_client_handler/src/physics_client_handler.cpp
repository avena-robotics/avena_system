#include "physics_client_handler/physics_client_handler.hpp"

namespace physics_client_handler
{
    PhysicsClientHandler::PhysicsClientHandler(const helpers::commons::RobotInfo &robot_info)
        : _robot_info(robot_info)
    {
        // Physics server handler initialization
        _bullet_client = std::make_shared<bullet_client::b3RobotSimulatorClientAPI>();
        if (!_bullet_client->connect(eCONNECT_SHARED_MEMORY))
            throw PhysicsClientHandlerError("Cannot connect to Bullet physics server");
        _bullet_client->syncBodies();
        _readSceneInfoFromPhysicsServer(_robot_info);
    }

    PhysicsClientHandler::~PhysicsClientHandler()
    {
        if (_bullet_client->isConnected())
            _bullet_client->disconnect();
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
        for (size_t i = 0; i < _joint_handles.size(); i++)
            _bullet_client->resetJointState(_robot_idx, _joint_handles[i], joint_states[i]);
    }

    void PhysicsClientHandler::_readSceneInfoFromPhysicsServer(const helpers::commons::RobotInfo &robot_info)
    {
        _bullet_client->syncBodies();
        int num_bodies = _bullet_client->getNumBodies();
        if (num_bodies == 0)
            throw PhysicsClientHandlerError("There are no objects in the physics server");

        _joint_handles.clear();
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
                            _joint_handles.push_back(joint_idx);
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

} // namespace physics_client_handler
