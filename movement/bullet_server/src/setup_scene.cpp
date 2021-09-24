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

        bullet_client::b3RobotSimulatorClientAPI::SharedPtr sim = std::make_shared<bullet_client::b3RobotSimulatorClientAPI>();
        bool connected = sim->connect(eCONNECT_SHARED_MEMORY);
        if (!connected)
        {
            RCLCPP_ERROR(get_logger(), "Cannot connect to physics server");
            return ReturnCode::FAILURE;
        }

        sim->setTimeOut(10);
        sim->syncBodies();
        sim->setGravity(btVector3(0, 0, -9.81));

        // --- Robot ---
        // Read robot description and base transformation to world
        const std::string robot_urdf = helpers::commons::getRobotDescription();
        if (robot_urdf.empty())
        {
            RCLCPP_ERROR(get_logger(), "Cannot read robot description");
            return ReturnCode::FAILURE;
        }
        const auto bullet_server_package_dir = std::filesystem::path(ament_index_cpp::get_package_share_directory("bullet_server"));
        const auto arm_urdf_path = bullet_server_package_dir / "robot_description.urdf";
        std::ofstream f(arm_urdf_path);
        f << robot_urdf;
        f.close();
        b3RobotSimulatorLoadUrdfFileArgs urdf_load_args;
        // Load transformation for arm base
        geometry_msgs::msg::TransformStamped tf_to_base;
        RCLCPP_INFO_STREAM(get_logger(), "Reading transform from \"world\" to \"" << _robot_info.base_link_name << "\"");
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
        // urdf_load_args.m_flags = URDF_USE_SELF_COLLISION;
        int robot_id = sim->loadURDF(arm_urdf_path, urdf_load_args);
        RCLCPP_DEBUG_STREAM(get_logger(), "Loaded robot with unique ID: " << robot_id);

        // ////////////////////////////////////////////////////////////////////////////////////////////
        // Loading static collision scene
        // Parse xacro file with world description
        const auto avena_bringup_shared_dir = std::filesystem::path(ament_index_cpp::get_package_share_directory("avena_bringup"));
        const auto world_xacro_path = avena_bringup_shared_dir / "worlds" / "avena_table.urdf.xacro";
        const auto parsed_urdf_path = bullet_server_package_dir / "avena_table.urdf";
        std::string parsing_command = "xacro -o " + parsed_urdf_path.string() +
                                      " x_min:=" + std::to_string(_workspace_area.x_min) +
                                      " x_max:=" + std::to_string(_workspace_area.x_max) +
                                      " y_min:=" + std::to_string(_workspace_area.y_min) +
                                      " y_max:=" + std::to_string(_workspace_area.y_max) +
                                      " " + world_xacro_path.string();
        RCLCPP_INFO_STREAM(get_logger(), "Command to parse xacro: \"" << parsing_command << "\"");
        int code = system(parsing_command.c_str());
        if (code != 0)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Cannot parse xacro file. Error code: " << code << ". Exiting...");
            return ReturnCode::FAILURE;
        }
        int table_id = sim->loadURDF(parsed_urdf_path);
        RCLCPP_DEBUG_STREAM(get_logger(), "Loaded table with unique ID: " << table_id);

        sim->disconnect();
        RCLCPP_INFO(get_logger(), "World created successfully");
        return ReturnCode::SUCCESS;
    }

} // namespace bullet_server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(bullet_server::SetupScene)