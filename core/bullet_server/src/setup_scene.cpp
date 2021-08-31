#include "bullet_server/setup_scene.hpp"

namespace bullet_server
{

    SetupScene::SetupScene(const rclcpp::NodeOptions &options)
        : Node("scene_setup", options)
    {
        helpers::commons::setLoggerLevelFromParameter(this);
        status = custom_interfaces::msg::Heartbeat::STOPPED;
        _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
    }

    SetupScene::~SetupScene()
    {
        shutDownNode();
    }

    void SetupScene::initNode()
    {
        RCLCPP_DEBUG(get_logger(), "Initializing node");
        status = custom_interfaces::msg::Heartbeat::STARTING;
        if (_createWorld() != ReturnCode::SUCCESS)
        {
            RCLCPP_WARN(get_logger(), "Error occured while initializing node");
            status = custom_interfaces::msg::Heartbeat::STOPPED;
            return;
        }
        status = custom_interfaces::msg::Heartbeat::RUNNING;
    }

    void SetupScene::shutDownNode()
    {
        RCLCPP_DEBUG(get_logger(), "Shutting down node");
        status = custom_interfaces::msg::Heartbeat::STOPPING;
        // TODO: Remove everything from world,
        bullet_client::b3RobotSimulatorClientAPI::SharedPtr sim(new bullet_client::b3RobotSimulatorClientAPI);
        bool connected = sim->connect(eCONNECT_SHARED_MEMORY);
        if (!connected)
        {
            RCLCPP_ERROR(get_logger(), "Cannot connect to physics server");
            return;
        }
        sim->syncBodies();
        sim->removeAllUserDebugItems();
        int num_bodies = sim->getNumBodies();
        RCLCPP_DEBUG_STREAM(get_logger(), "Number of bodies: " << num_bodies);
        for (int body_id = 0; body_id < num_bodies; body_id++)
        {
            int unique_body_id = body_id;
            // int unique_body_id = sim->getBodyUniqueId(body_id);
            b3BodyInfo body_info;
            sim->getBodyInfo(unique_body_id, &body_info);
            std::stringstream ss;
            ss << "Unique body ID: " << unique_body_id
               << ", body name: \"" << body_info.m_bodyName
               << "\", base name: \"" << body_info.m_baseName << "\"";
            RCLCPP_DEBUG(get_logger(), ss.str());
            sim->removeBody(unique_body_id);
        }

        status = custom_interfaces::msg::Heartbeat::STOPPED;
    }

    ReturnCode SetupScene::_getParametersFromServer()
    {
        RCLCPP_INFO_ONCE(get_logger(), "Reading parameters from the server");

        nlohmann::json area = helpers::commons::getParameter("areas");
        if (area.empty())
            return ReturnCode::FAILURE;

        RCLCPP_DEBUG(get_logger(), "Reading table area dimensions");
        _workspace_area.x_min = area["table_area"]["min"]["x"].get<float>();
        _workspace_area.y_min = area["table_area"]["min"]["y"].get<float>();
        _workspace_area.z_min = area["table_area"]["min"]["z"].get<float>();

        _workspace_area.x_max = area["table_area"]["max"]["x"].get<float>();
        _workspace_area.y_max = area["table_area"]["max"]["y"].get<float>();
        _workspace_area.z_max = area["table_area"]["max"]["z"].get<float>();

        RCLCPP_DEBUG(get_logger(), "Reading robot information");
        if (auto robot_info = helpers::commons::getRobotInfo())
            _robot_info = *robot_info;
        else
            return ReturnCode::FAILURE;

        RCLCPP_INFO(get_logger(), "Parameters read successfully...");
        return ReturnCode::SUCCESS;
    }

    ReturnCode SetupScene::_createWorld()
    {
        RCLCPP_INFO(get_logger(), "Creating brain world");

        if (_getParametersFromServer() != ReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Cannot read parameters from server");
            return ReturnCode::FAILURE;
        }

        bullet_client::b3RobotSimulatorClientAPI::SharedPtr sim(new bullet_client::b3RobotSimulatorClientAPI);
        bool connected = sim->connect(eCONNECT_SHARED_MEMORY);
        if (!connected)
        {
            RCLCPP_ERROR(get_logger(), "Cannot connect to physics server");
            return ReturnCode::FAILURE;
        }

        // Table dimensions
        const double TABLE_HEIGHT = 0.01;
        const double TABLE_LENGTH_HALF = (_workspace_area.y_max - _workspace_area.y_min) / 2;
        const double TABLE_WIDTH_HALF = (_workspace_area.x_max - _workspace_area.x_min) / 2;

        // sim->configureDebugVisualizer(COV_ENABLE_GUI, 1);
        // sim->configureDebugVisualizer( COV_ENABLE_SHADOWS, 0);//COV_ENABLE_WIREFRAME
        sim->setTimeOut(10);
        //syncBodies is only needed when connecting to an existing physics server that has already some bodies
        sim->syncBodies();
        // btScalar fixedTimeStep = 1. / 240.;
        sim->setGravity(btVector3(0, 0, -9.81));

        // --- Robot ---
        // Read robot description and base transformation to world
        const std::string robot_urdf = helpers::commons::getRobotDescription();
        if (robot_urdf.empty())
        {
            RCLCPP_ERROR(get_logger(), "Cannot read robot description");
            return ReturnCode::FAILURE;
        }
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("bullet_server");
        const std::string arm_urdf_path = package_share_directory + "/robot_description.urdf";
        std::ofstream f(arm_urdf_path);
        f << robot_urdf;
        f.close();
        b3RobotSimulatorLoadUrdfFileArgs urdf_load_args;
        // Load transformation for arm base
        geometry_msgs::msg::TransformStamped tf_to_base;
        if (auto tf_to_base_opt = helpers::vision::getTransformStamped("world", _robot_info.base_link_name))
        {
            RCLCPP_INFO(get_logger(), "Transformation from base to world read successfully");
            tf_to_base = *tf_to_base_opt;
            urdf_load_args.m_startPosition = btVector3(tf_to_base.transform.translation.x, tf_to_base.transform.translation.y, tf_to_base.transform.translation.z);
            urdf_load_args.m_startOrientation = btQuaternion(tf_to_base.transform.rotation.x, tf_to_base.transform.rotation.y, tf_to_base.transform.rotation.z, tf_to_base.transform.rotation.w);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Invalid transformation from robot base to world. Exiting...");
            return ReturnCode::FAILURE;
        }

        urdf_load_args.m_forceOverrideFixedBase = true;
        // urdf_load_args.m_flags = URDF_USE_SELF_COLLISION_EXCLUDE_PARENT;
        int robot_id = sim->loadURDF(arm_urdf_path, urdf_load_args);
        RCLCPP_DEBUG_STREAM(get_logger(), "Loaded robot unique ID: " << robot_id);

        ////////////////////////////////////////////////////////////////////////////////////////////
        // --- Table ---
        b3RobotSimulatorCreateCollisionShapeArgs table_collision_args;
        table_collision_args.m_halfExtents = btVector3(TABLE_WIDTH_HALF, TABLE_LENGTH_HALF, TABLE_HEIGHT / 2);
        int table_collision = sim->createCollisionShape(GEOM_BOX, table_collision_args);
        b3RobotSimulatorCreateVisualShapeArgs table_visual_args;
        table_visual_args.m_halfExtents = btVector3(TABLE_WIDTH_HALF, TABLE_LENGTH_HALF, TABLE_HEIGHT / 2);
        int table_visual = sim->createVisualShape(GEOM_BOX, table_visual_args);

        // --- Side posts for cameras ---
        b3RobotSimulatorCreateCollisionShapeArgs stick1_collision_args;
        stick1_collision_args.m_halfExtents = btVector3(0.02, 0.02, 0.7);
        int stick1_collision = sim->createCollisionShape(GEOM_BOX, stick1_collision_args);
        int stick2_collision = sim->createCollisionShape(GEOM_BOX, stick1_collision_args);
        b3RobotSimulatorCreateVisualShapeArgs stick1_visual_args;
        stick1_visual_args.m_halfExtents = btVector3(0.02, 0.02, 0.7);
        int stick1_visual = sim->createVisualShape(GEOM_BOX, stick1_visual_args);
        int stick2_visual = sim->createVisualShape(GEOM_BOX, stick1_visual_args);

        // --- Cameras ---
        b3RobotSimulatorCreateCollisionShapeArgs camera_collision_args;
        camera_collision_args.m_halfExtents = btVector3(0.05, 0.05, 0.05);
        int camera1_collision = sim->createCollisionShape(GEOM_BOX, camera_collision_args);
        int camera2_collision = sim->createCollisionShape(GEOM_BOX, camera_collision_args);
        b3RobotSimulatorCreateVisualShapeArgs camera_visual_args;
        camera_visual_args.m_halfExtents = btVector3(0.05, 0.05, 0.05);
        int camera1_visual = sim->createVisualShape(GEOM_BOX, camera_visual_args);
        int camera2_visual = sim->createVisualShape(GEOM_BOX, camera_visual_args);

        // --- Collision walls ---
        const float collision_wall_height = 1;
        const float wall_thickness = 0.01;
        // Front and back wall
        b3RobotSimulatorCreateCollisionShapeArgs plane_front_collision_args;
        plane_front_collision_args.m_halfExtents = btVector3(wall_thickness / 2, TABLE_LENGTH_HALF, collision_wall_height / 2);
        int front_plane_collision = sim->createCollisionShape(GEOM_BOX, plane_front_collision_args);
        int front_plane2_collision = sim->createCollisionShape(GEOM_BOX, plane_front_collision_args);
        b3RobotSimulatorCreateVisualShapeArgs plane_front_visual_args;
        plane_front_visual_args.m_halfExtents = btVector3(wall_thickness / 2, TABLE_LENGTH_HALF, collision_wall_height / 2);
        int front_plane_visual = sim->createVisualShape(GEOM_BOX, plane_front_visual_args);
        int front_plane2_visual = sim->createVisualShape(GEOM_BOX, plane_front_visual_args);
        // Side walls
        b3RobotSimulatorCreateCollisionShapeArgs plane_side_collision_args;
        plane_side_collision_args.m_halfExtents = btVector3(TABLE_WIDTH_HALF, wall_thickness / 2, collision_wall_height / 2);
        int side_plane_collision = sim->createCollisionShape(GEOM_BOX, plane_side_collision_args);
        int side_plane2_collision = sim->createCollisionShape(GEOM_BOX, plane_side_collision_args);
        b3RobotSimulatorCreateVisualShapeArgs plane_side_visual_args;
        plane_side_visual_args.m_halfExtents = btVector3(TABLE_WIDTH_HALF, wall_thickness / 2, collision_wall_height / 2);
        int side_plane_visual = sim->createVisualShape(GEOM_BOX, plane_side_visual_args);
        int side_plane2_visual = sim->createVisualShape(GEOM_BOX, plane_side_visual_args);
        // Top wall
        b3RobotSimulatorCreateCollisionShapeArgs plane_top_collision_args;
        plane_top_collision_args.m_halfExtents = btVector3(TABLE_WIDTH_HALF, TABLE_LENGTH_HALF, wall_thickness / 2);
        int top_plane_collision = sim->createCollisionShape(GEOM_BOX, plane_top_collision_args);
        b3RobotSimulatorCreateVisualShapeArgs plane_top_visual_args;
        plane_top_visual_args.m_halfExtents = btVector3(TABLE_WIDTH_HALF, TABLE_LENGTH_HALF, wall_thickness / 2);
        int top_plane_visual = sim->createVisualShape(GEOM_BOX, plane_top_visual_args);

        ////////////////////////////////////////////////////////////////////////////
        // TODO: This should be removed later when whole real scene is setup properly
        int avena_arm_base_collision = -1;
        int avena_arm_base_visual = -1;
        if (_robot_info.robot_name == "franka")
        {
            // --- Avena arm base ---
            b3RobotSimulatorCreateCollisionShapeArgs avena_arm_base_collision_args;
            avena_arm_base_collision_args.m_halfExtents = btVector3(0.1, 0.1, 0.05);
            avena_arm_base_collision = sim->createCollisionShape(GEOM_BOX, avena_arm_base_collision_args);
            b3RobotSimulatorCreateVisualShapeArgs avena_arm_base_visual_args;
            avena_arm_base_visual_args.m_halfExtents = btVector3(0.1, 0.1, 0.05);
            avena_arm_base_visual = sim->createVisualShape(GEOM_BOX, avena_arm_base_visual_args);
        }
        ////////////////////////////////////////////////////////////////////////////

        // Create collision table with all parts
        b3RobotSimulatorCreateMultiBodyArgs table_args;
        table_args.m_baseMass = 0;
        table_args.m_basePosition = btVector3(TABLE_WIDTH_HALF, 0, -TABLE_HEIGHT / 2);
        table_args.m_baseVisualShapeIndex = table_visual;
        table_args.m_baseCollisionShapeIndex = table_collision;
        std::vector<int> collision_indices = {
            front_plane_collision,
            front_plane2_collision,
            side_plane_collision,
            side_plane2_collision,
            top_plane_collision,
            stick1_collision,
            stick2_collision,
            camera1_collision,
            camera2_collision,
        };

        ////////////////////////////////////////////////////////////////////////////
        // TODO: This should be removed later when whole real scene is setup properly
        if (_robot_info.robot_name == "franka")
        {
            collision_indices.push_back(avena_arm_base_collision);
        }
        ////////////////////////////////////////////////////////////////////////////

        table_args.m_numLinks = collision_indices.size();
        table_args.m_linkCollisionShapeIndices = collision_indices.data(); //!1
        std::vector<int> visual_indices = {
            front_plane_visual,
            front_plane2_visual,
            side_plane_visual,
            side_plane2_visual,
            top_plane_visual,
            stick1_visual,
            stick2_visual,
            camera1_visual,
            camera2_visual,
        };

        ////////////////////////////////////////////////////////////////////////////
        // TODO: This should be removed later when whole real scene is setup properly
        if (_robot_info.robot_name == "franka")
        {
            visual_indices.push_back(avena_arm_base_visual);
        }
        ////////////////////////////////////////////////////////////////////////////

        table_args.m_linkVisualShapeIndices = visual_indices.data(); //!2
        std::vector<double> mass_links(collision_indices.size(), 1.0);
        table_args.m_linkMasses = mass_links.data(); //!3

        std::vector<btVector3> link_positions = {
            btVector3(-TABLE_WIDTH_HALF - 2 * 0.005, 0, collision_wall_height / 2),
            btVector3(TABLE_WIDTH_HALF + 2 * 0.005 + 0.2, 0, collision_wall_height / 2),
            btVector3(0, -TABLE_LENGTH_HALF - 2 * 0.005, collision_wall_height / 2),
            btVector3(0, TABLE_LENGTH_HALF + 2 * 0.005, collision_wall_height / 2),
            btVector3(0, 0, 1.05 + wall_thickness / 2),
            btVector3(0.38, 1.18, 0.7),
            btVector3(0.38, -1.18, 0.7),
            btVector3(0.35, 1.15, 1.05),
            btVector3(0.35, -1.15, 1.05),
            // btVector3(0.18 - TABLE_WIDTH_HALF, 0.35, 0.05),
        };

        ////////////////////////////////////////////////////////////////////////////
        // TODO: This should be removed later when whole real scene is setup properly
        if (_robot_info.robot_name == "franka")
        {
            link_positions.push_back(btVector3(0.18 - TABLE_WIDTH_HALF, -tf_to_base.transform.translation.y, 0.05));
        }
        ////////////////////////////////////////////////////////////////////////////

        table_args.m_linkPositions = link_positions.data(); //!4
        std::vector<btQuaternion> link_orientations(collision_indices.size(), btQuaternion(0, 0, 0, 1));
        table_args.m_linkOrientations = link_orientations.data(); //!5
        //! To chceck faster - inertial same positions
        table_args.m_linkInertialFramePositions = link_positions.data();       //!6
        table_args.m_linkInertialFrameOrientations = link_orientations.data(); //!7
        std::vector<int> link_parent_indices(link_positions.size(), 0);
        table_args.m_linkParentIndices = link_parent_indices.data(); //!8
        std::vector<int> link_joint_types(link_positions.size(), JointType::eFixedType);
        table_args.m_linkJointTypes = link_joint_types.data(); //!9
        std::vector<btVector3> link_joint_axes(link_positions.size(), btVector3(0, 0, 1));
        table_args.m_linkJointAxes = link_joint_axes.data(); //!10

        // Change opacity for collision walls
        int table_id = sim->createMultiBody(table_args);
        RCLCPP_DEBUG_STREAM(get_logger(), "Table unique ID: " << table_id);
        b3RobotSimulatorChangeVisualShapeArgs visual;
        visual.m_rgbaColor = btVector4(0, 1, 0, 0.05);
        visual.m_objectUniqueId = table_id;
        visual.m_linkIndex = 0;
        visual.m_hasRgbaColor = true;
        sim->changeVisualShape(visual);
        visual.m_linkIndex = 1;
        sim->changeVisualShape(visual);
        visual.m_linkIndex = 2;
        sim->changeVisualShape(visual);
        visual.m_linkIndex = 3;
        sim->changeVisualShape(visual);
        visual.m_linkIndex = 4;
        sim->changeVisualShape(visual);

        ////////////////////////////////////////////////////////////////////////////////////////////

        // // Calibration mat params
        // b3RobotSimulatorCreateCollisionShapeArgs CAL_MAT_COLLISION_ARGS;
        // b3RobotSimulatorCreateVisualShapeArgs CAL_MAT_VISUAL_ARGS;
        // b3RobotSimulatorCreateMultiBodyArgs CAL_MAT_BODY_ARGS;
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
RCLCPP_COMPONENTS_REGISTER_NODE(bullet_server::SetupScene)