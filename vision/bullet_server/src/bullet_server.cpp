#include "bullet_server/bullet_server.hpp"

namespace bullet_server
{

    BulletServer::BulletServer(const rclcpp::NodeOptions &options)
        : Node("bullet_server", options)
    {
// Right now I do not know why Bullet needs this but for now keep it here
#ifndef _WIN32
        struct sigaction action;
        memset(&action, 0x0, sizeof(action));
        action.sa_handler = cleanup;
        static const int signos[] = {SIGHUP, SIGINT, SIGQUIT, SIGABRT, SIGSEGV, SIGPIPE, SIGTERM};
        for (size_t ii(0); ii < sizeof(signos) / sizeof(*signos); ++ii)
        {
            if (0 != sigaction(signos[ii], &action, NULL))
            {
                err(EXIT_FAILURE, "signal %d", signos[ii]);
            }
        }
#endif

        _server_thread = std::thread([]()
                                     {
                                         DummyGUIHelper noGfx;

                                         CommonExampleOptions commons_example_options(&noGfx);

                                         // args.GetCmdLineArgument("shared_memory_key", gSharedMemoryKey);
                                         // args.GetCmdLineArgument("sharedMemoryKey", gSharedMemoryKey);

                                         // commons_example_options.m_option |= PHYSICS_SERVER_ENABLE_COMMAND_LOGGING;
                                         // commons_example_options.m_option |= PHYSICS_SERVER_REPLAY_FROM_COMMAND_LOG;

                                         example = (SharedMemoryCommon *)PhysicsServerCreateFuncBullet2(commons_example_options);

                                         example->initPhysics();

                                         while (example->isConnected() && !(example->wantsTermination() || interrupted))
                                         {
                                             example->stepSimulation(1.f / 60.f);
                                         }

                                         example->exitPhysics();

                                         delete example;
                                     });
        _loading_scene_timer = create_wall_timer(std::chrono::milliseconds(500), std::bind(&BulletServer::_createWorld, this));
    }

    BulletServer::~BulletServer()
    {
        if (_server_thread.joinable())
            _server_thread.join();
    }

    ReturnCode BulletServer::_getParametersFromServer()
    {
        RCLCPP_INFO_ONCE(get_logger(), "Reading parameters from the server");

        nlohmann::json area = helpers::commons::getParameter("areas");
        if (area.empty())
            return ReturnCode::FAILURE;

        _workspace_area.x_min = area["table_area"]["min"]["x"].get<float>();
        _workspace_area.y_min = area["table_area"]["min"]["y"].get<float>();
        _workspace_area.z_min = area["table_area"]["min"]["z"].get<float>();

        _workspace_area.x_max = area["table_area"]["max"]["x"].get<float>();
        _workspace_area.y_max = area["table_area"]["max"]["y"].get<float>();
        _workspace_area.z_max = area["table_area"]["max"]["z"].get<float>();

        RCLCPP_INFO(get_logger(), "Parameters read successfully...");
        return ReturnCode::SUCCESS;
    }

    ReturnCode BulletServer::_createWorld()
    {
        RCLCPP_INFO(get_logger(), "Creating brain world");

        _loading_scene_timer->cancel();

        if (_getParametersFromServer() != ReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Cannot read parameters from server. Exiting...");
            rclcpp::shutdown();
            std::exit(-1);
        }

        bullet_client::b3RobotSimulatorClientAPI::SharedPtr sim(new bullet_client::b3RobotSimulatorClientAPI);
        bool connected = sim->connect(eCONNECT_SHARED_MEMORY);
        if (!connected)
        {
            RCLCPP_WARN(get_logger(), "Cannot connect to physics server");
            return ReturnCode::FAILURE;
        }

        //! robot params
        b3RobotSimulatorLoadUrdfFileArgs URDF_LOAD_PARAMS;

        const double TABLE_HEIGHT = 0.545;
        // const double TABLE_LENGTH_HALF = 1.2;
        // const double TABLE_WIDTH_HALF = 0.4;
        const double TABLE_LENGTH_HALF = (_workspace_area.y_max - _workspace_area.y_min) / 2;
        const double TABLE_WIDTH_HALF = (_workspace_area.x_max - _workspace_area.x_min) / 2;

        b3RobotSimulatorCreateCollisionShapeArgs TABLE_COLLISION_ARGS;
        b3RobotSimulatorCreateVisualShapeArgs TABLE_VISUAL_ARGS;
        b3RobotSimulatorCreateMultiBodyArgs TABLE_ARGS;
        b3RobotSimulatorCreateCollisionShapeArgs STICK1_COLLISION_ARGS;
        b3RobotSimulatorCreateVisualShapeArgs STICK1_VISUAL_ARGS;
        b3RobotSimulatorCreateCollisionShapeArgs CAMERA_COLLISION_ARGS;
        b3RobotSimulatorCreateVisualShapeArgs CAMERA_VISUAL_ARGS;

        //!scene boudings params
        b3RobotSimulatorCreateCollisionShapeArgs PLANE_FRONT_COLLISION_ARGS;
        b3RobotSimulatorCreateVisualShapeArgs PLANE_FRONT_VISUAL_ARGS;
        b3RobotSimulatorCreateCollisionShapeArgs PLANE_SIDE_COLLISION_ARGS;
        b3RobotSimulatorCreateVisualShapeArgs PLANE_SIDE_VISUAL_ARGS;

        //! calibration mat params
        b3RobotSimulatorCreateCollisionShapeArgs CAL_MAT_COLLISION_ARGS;
        b3RobotSimulatorCreateVisualShapeArgs CAL_MAT_VISUAL_ARGS;
        b3RobotSimulatorCreateMultiBodyArgs CAL_MAT_BODY_ARGS;

        // sim->configureDebugVisualizer(COV_ENABLE_GUI, 1);
        //	sim->configureDebugVisualizer( COV_ENABLE_SHADOWS, 0);//COV_ENABLE_WIREFRAME
        sim->setTimeOut(10);
        //syncBodies is only needed when connecting to an existing physics server that has already some bodies
        sim->syncBodies();
        // btScalar fixedTimeStep = 1. / 240.;
        btQuaternion q = sim->getQuaternionFromEuler(btVector3(0.1, 0.2, 0.3));
        btVector3 rpy;
        rpy = sim->getEulerFromQuaternion(q);

        sim->setGravity(btVector3(0, 0, -9.81));

        // RCLCPP_INFO(get_logger(), "Spawning plane");
        // sim->loadURDF("/home/avena/repos/bullet3-3.17/data/plane.urdf");

        //! LOAD AVENA ARM FROM URDF
        URDF_LOAD_PARAMS.m_forceOverrideFixedBase = true;
        URDF_LOAD_PARAMS.m_flags = URDF_USE_SELF_COLLISION_EXCLUDE_PARENT;
        URDF_LOAD_PARAMS.m_startPosition = btVector3(0.18, -0.35, TABLE_HEIGHT + 0.05);
        URDF_LOAD_PARAMS.m_startOrientation = sim->getQuaternionFromEuler(btVector3(0, 0, 0));

        const std::string robot_urdf = helpers::commons::getRobotDescription();
        const std::string AVENA_ARM_URDF_PATH = "robot_description.urdf";
        std::ofstream f(AVENA_ARM_URDF_PATH);
        f << robot_urdf;
        f.close();
        sim->loadURDF(AVENA_ARM_URDF_PATH, URDF_LOAD_PARAMS);

        ////////////////////////////////////////////////////////////////////////////////////////////
        //! LOAD THE SCENE
        TABLE_COLLISION_ARGS.m_halfExtents = btVector3(TABLE_WIDTH_HALF, TABLE_LENGTH_HALF, TABLE_HEIGHT / 2.0);
        int table_collision = sim->createCollisionShape(GEOM_BOX, TABLE_COLLISION_ARGS);
        TABLE_VISUAL_ARGS.m_halfExtents = btVector3(TABLE_WIDTH_HALF, TABLE_LENGTH_HALF, TABLE_HEIGHT / 2.0);
        int table_visual = sim->createVisualShape(GEOM_BOX, TABLE_VISUAL_ARGS);

        //? Try to create other elements and link them with the table (which is the parent body)
        STICK1_COLLISION_ARGS.m_halfExtents = btVector3(0.02, 0.02, 0.3);
        int stick1_collision = sim->createCollisionShape(GEOM_BOX, STICK1_COLLISION_ARGS);
        int stick2_collision = sim->createCollisionShape(GEOM_BOX, STICK1_COLLISION_ARGS);
        STICK1_VISUAL_ARGS.m_halfExtents = btVector3(0.02, 0.02, 0.3);
        int stick1_visual = sim->createVisualShape(GEOM_BOX, STICK1_VISUAL_ARGS);
        int stick2_visual = sim->createVisualShape(GEOM_BOX, STICK1_VISUAL_ARGS);

        CAMERA_COLLISION_ARGS.m_halfExtents = btVector3(0.05, 0.05, 0.05);
        int camera_collision = sim->createCollisionShape(GEOM_BOX, CAMERA_COLLISION_ARGS);
        int camera2_collision = sim->createCollisionShape(GEOM_BOX, CAMERA_COLLISION_ARGS);
        CAMERA_VISUAL_ARGS.m_halfExtents = btVector3(0.05, 0.05, 0.05);
        int camera_visual = sim->createVisualShape(GEOM_BOX, CAMERA_VISUAL_ARGS);
        int camera2_visual = sim->createVisualShape(GEOM_BOX, CAMERA_VISUAL_ARGS);

        PLANE_FRONT_COLLISION_ARGS.m_halfExtents = btVector3(0.005, TABLE_LENGTH_HALF, 0.5);
        int front_plane_collision = sim->createCollisionShape(GEOM_BOX, PLANE_FRONT_COLLISION_ARGS);
        int front_plane2_collision = sim->createCollisionShape(GEOM_BOX, PLANE_FRONT_COLLISION_ARGS);
        PLANE_FRONT_VISUAL_ARGS.m_halfExtents = btVector3(0.005, TABLE_LENGTH_HALF, 0.5);
        int front_plane_visual = sim->createVisualShape(GEOM_BOX, PLANE_FRONT_VISUAL_ARGS);
        int front_plane2_visual = sim->createVisualShape(GEOM_BOX, PLANE_FRONT_VISUAL_ARGS);

        PLANE_SIDE_COLLISION_ARGS.m_halfExtents = btVector3(TABLE_WIDTH_HALF, 0.005, 0.5);
        int side_plane_collision = sim->createCollisionShape(GEOM_BOX, PLANE_SIDE_COLLISION_ARGS);
        int side_plane2_collision = sim->createCollisionShape(GEOM_BOX, PLANE_SIDE_COLLISION_ARGS);

        PLANE_SIDE_VISUAL_ARGS.m_halfExtents = btVector3(TABLE_WIDTH_HALF, 0.005, 0.5);
        int side_plane_visual = sim->createVisualShape(GEOM_BOX, PLANE_SIDE_VISUAL_ARGS);
        int side_plane2_visual = sim->createVisualShape(GEOM_BOX, PLANE_SIDE_VISUAL_ARGS);

        TABLE_ARGS.m_baseMass = 0;
        TABLE_ARGS.m_basePosition = btVector3(0.4, 0, TABLE_HEIGHT / 2.0);
        TABLE_ARGS.m_baseVisualShapeIndex = table_visual;
        TABLE_ARGS.m_baseCollisionShapeIndex = table_collision;

        std::vector<int> collision_indices = {stick1_collision, stick2_collision, camera_collision, camera2_collision, front_plane_collision, front_plane2_collision, side_plane_collision, side_plane2_collision};
        TABLE_ARGS.m_numLinks = collision_indices.size();
        TABLE_ARGS.m_linkCollisionShapeIndices = collision_indices.data(); //!1
        std::vector<int> visual_indices = {stick1_visual, stick2_visual, camera_visual, camera2_visual, front_plane_visual, front_plane2_visual, side_plane_visual, side_plane2_visual};
        TABLE_ARGS.m_linkVisualShapeIndices = visual_indices.data(); //!2
        std::vector<double> mass_links = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        TABLE_ARGS.m_linkMasses = mass_links.data(); //!3

        std::vector<btVector3> link_positions = {btVector3(0.38, 1.18, TABLE_HEIGHT / 2.0 + 0.30), btVector3(0.38, -1.18, TABLE_HEIGHT / 2.0 + 0.30), btVector3(-0.03, -0.03, 0.3 + 0.05), btVector3(-0.03, 0.03, 0.3 + 0.05),
                                                 btVector3(-TABLE_WIDTH_HALF - 2 * 0.005, 0, TABLE_HEIGHT + 0.2), btVector3(TABLE_WIDTH_HALF + 2 * 0.005, 0, TABLE_HEIGHT + 0.2), btVector3(0, -TABLE_LENGTH_HALF - 2 * 0.005, TABLE_HEIGHT + 0.2),
                                                 btVector3(0, TABLE_LENGTH_HALF + 2 * 0.005, TABLE_HEIGHT + 0.2)};
        // std::vector<btVector3> link_positions = {btVector3(0, 0, 1)};
        TABLE_ARGS.m_linkPositions = link_positions.data(); //!4
        std::vector<btQuaternion> link_orientations = {btQuaternion(0, 0, 0, 1), btQuaternion(0, 0, 0, 1), btQuaternion(0, 0, 0, 1), btQuaternion(0, 0, 0, 1), btQuaternion(0, 0, 0, 1), btQuaternion(0, 0, 0, 1),
                                                       btQuaternion(0, 0, 0, 1), btQuaternion(0, 0, 0, 1)};
        // std::vector<btQuaternion> link_orientations = {btQuaternion(0, 0, 0, 1)};
        TABLE_ARGS.m_linkOrientations = link_orientations.data(); //!5
        //! To chceck faster - inertial same positions
        TABLE_ARGS.m_linkInertialFramePositions = link_positions.data();       //!6
        TABLE_ARGS.m_linkInertialFrameOrientations = link_orientations.data(); //!7
        std::vector<int> link_parent_indices = {0, 0, 1, 2, 0, 0, 0, 0};
        // std::vector<int> link_parent_indices = {0};
        TABLE_ARGS.m_linkParentIndices = link_parent_indices.data(); //!8
        std::vector<int> link_joint_types = {4, 4, 4, 4, 4, 4, 4, 4};
        // std::vector<int> link_joint_types = {4};
        TABLE_ARGS.m_linkJointTypes = link_joint_types.data(); //!9
        std::vector<btVector3> link_joint_axes = {btVector3(0, 0, 1), btVector3(0, 0, 1), btVector3(0, 0, 1), btVector3(0, 0, 1), btVector3(0, 0, 1), btVector3(0, 0, 1), btVector3(0, 0, 1), btVector3(0, 0, 1)};
        // std::vector<btVector3> link_joint_axes = {btVector3(0, 0, 1)};
        TABLE_ARGS.m_linkJointAxes = link_joint_axes.data(); //!10

        int table_id = sim->createMultiBody(TABLE_ARGS);
        b3RobotSimulatorChangeVisualShapeArgs visual_;
        visual_.m_rgbaColor = btVector4(0, 1, 0, 0.1);
        visual_.m_objectUniqueId = table_id;
        visual_.m_linkIndex = 4;
        visual_.m_hasRgbaColor = true;
        sim->changeVisualShape(visual_);
        visual_.m_linkIndex = 5;
        sim->changeVisualShape(visual_);
        visual_.m_linkIndex = 6;
        sim->changeVisualShape(visual_);
        visual_.m_linkIndex = 7;
        sim->changeVisualShape(visual_);
        ////////////////////////////////////////////////////////////////////////////////////////////

        // // Calibration mat params
        // CAL_MAT_COLLISION_ARGS.m_halfExtents = btVector3(0.21 / 2.0, 0.17 / 2.0, 0.02306 / 2.0);
        // int cal_mat_col = sim->createCollisionShape(GEOM_BOX, CAL_MAT_COLLISION_ARGS);
        // CAL_MAT_VISUAL_ARGS.m_halfExtents = btVector3(0.21 / 2.0, 0.17 / 2.0, 0.02306 / 2.0);
        // int cal_mat_vis = sim->createVisualShape(GEOM_BOX, CAL_MAT_VISUAL_ARGS);
        // CAL_MAT_BODY_ARGS.m_baseVisualShapeIndex = cal_mat_vis;
        // CAL_MAT_BODY_ARGS.m_baseCollisionShapeIndex = cal_mat_col;
        // CAL_MAT_BODY_ARGS.m_baseMass = 0.01;
        // CAL_MAT_BODY_ARGS.m_basePosition = btVector3(0, 0, 2);

        // int cal_mat_id = sim->createMultiBody(CAL_MAT_BODY_ARGS);

        // b3JointInfo joint_info;
        // joint_info.m_jointType = JointType::eFixedType;

        // joint_info.m_jointAxis[0] = 0;
        // joint_info.m_jointAxis[1] = 0;
        // joint_info.m_jointAxis[2] = 0;

        // btVector3 orient(0, 0, 3.14 / 4.0);
        // btQuaternion orient_ = sim->getQuaternionFromEuler(orient);

        // joint_info.m_parentFrame[0] = 0;
        // joint_info.m_parentFrame[1] = 0;
        // joint_info.m_parentFrame[2] = 0.11;
        // joint_info.m_parentFrame[3] = orient_[0];
        // joint_info.m_parentFrame[4] = orient_[1];
        // joint_info.m_parentFrame[5] = orient_[2];
        // joint_info.m_parentFrame[6] = orient_[3];

        // // b3LinkState state;
        // // sim->getLinkState(robot_arm, 7, 0, 0, &state);
        // // std::cout<<"x:"<<state.m_worldPosition[0]<<"   y:"<<state.m_worldPosition[1]<<"   z:"<<state.m_worldPosition[2]<<std::endl;
        // joint_info.m_childFrame[0] = 0;
        // joint_info.m_childFrame[1] = 0;
        // joint_info.m_childFrame[2] = 0;
        // joint_info.m_childFrame[3] = 0;
        // joint_info.m_childFrame[4] = 0;
        // joint_info.m_childFrame[5] = 0;
        // joint_info.m_childFrame[6] = 1;

        // //! Add axes visualization to the end efector
        // constexpr int END_EFECTOR_LINK_INDEX = 6; // TODO: Should be more dynamic
        // b3RobotSimulatorAddUserDebugLineArgs LINE_ARGS;
        // LINE_ARGS.m_parentObjectUniqueId = robot_arm;
        // LINE_ARGS.m_parentLinkIndex = END_EFECTOR_LINK_INDEX;
        // LINE_ARGS.m_colorRGB[0] = 1;
        // LINE_ARGS.m_colorRGB[1] = 0;
        // LINE_ARGS.m_colorRGB[2] = 0;
        // btVector3 from(0, 0, 0);
        // btVector3 to_x(0.5, 0, 0);
        // sim->addUserDebugLine(from, to_x, LINE_ARGS);
        // LINE_ARGS.m_colorRGB[0] = 0;
        // LINE_ARGS.m_colorRGB[1] = 1;
        // LINE_ARGS.m_colorRGB[2] = 0;
        // btVector3 to_y(0, 0.5, 0);
        // sim->addUserDebugLine(from, to_y, LINE_ARGS);
        // LINE_ARGS.m_colorRGB[0] = 0;
        // LINE_ARGS.m_colorRGB[1] = 0;
        // LINE_ARGS.m_colorRGB[2] = 1;
        // btVector3 to_z(0, 0, 0.5);
        // sim->addUserDebugLine(from, to_z, LINE_ARGS);
        // sim->configureDebugVisualizer(COV_ENABLE_RGB_BUFFER_PREVIEW, 1);

        // sim->resetJointState(robot_arm, 5, 2);
        //! Constrained body is not a part of the robot itself
        // std::cout << std::endl
        //           << "Constraints creator returned: " << sim->createConstraint(robot_arm, 6, cal_mat_id, -1, &joint_info) << std::endl;

        sim->disconnect();
        RCLCPP_INFO(get_logger(), "World created successfully");
        return ReturnCode::SUCCESS;
    }

} // namespace bullet_server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(bullet_server::BulletServer)