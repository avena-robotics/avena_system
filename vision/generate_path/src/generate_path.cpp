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
                                                                                 RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Received joint states [message throttles with 0.1 sec]");
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
        RCLCPP_INFO(get_logger(), "Generating pose path");
        _bullet_client->syncBodies();
        auto result = std::make_shared<GeneratePathPose::Result>();

        _setCurrentJointStatesOnPhysicsServer(_current_joint_states);

        // if (!_generatePath(Job::POSE))
        // {
        //     RCLCPP_ERROR(get_logger(), "Generate path to pose aborted.");
        //     goal_handle->abort(result);
        //     return;
        // }

        std::vector<float> joint_states(_current_joint_states->position.begin(), _current_joint_states->position.end());
        std::vector<float> goal_states = {0, 0, 0, 0, 0, 2, 0};
        const float safety_range = 0.003;
        const int threshold = 1;
        std::vector<int> obstacles = {_table_idx};
        std::vector<int> robot = {_robot_idx};
        auto path = rrt(joint_states, goal_states, 10000, 0.1, 0.01, _bullet_client.get(), safety_range, obstacles, robot, threshold);
        std::cout << path.size() << std::endl;
        
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

    ReturnCode GeneratePath::_setCurrentJointStatesOnPhysicsServer(const sensor_msgs::msg::JointState::SharedPtr &joint_states)
    {
        int num_bodies = _bullet_client->getNumBodies();
        for (int body_id = 0; body_id < num_bodies; ++body_id)
        {
            int body_unique_id = _bullet_client->getBodyUniqueId(body_id);
            b3BodyInfo body_info;
            _bullet_client->getBodyInfo(body_unique_id, &body_info);

            std::cout << "m_baseName: " << body_info.m_baseName << ", m_bodyName: " << body_info.m_bodyName << std::endl;

            if (std::strcmp(body_info.m_bodyName, _robot_info.robot_name.c_str()) == 0)
            {
                _robot_idx = body_unique_id;
                // std::cout << "body id: " << body_id << std::endl;
                // std::cout << "body unique id: " << body_unique_id << std::endl;
                // std::cout << "m_baseName: " << body_info.m_baseName << ", m_bodyName: " << body_info.m_bodyName << std::endl;

                int num_joints = _bullet_client->getNumJoints(body_unique_id);
                // std::cout << "num_joints: " << num_joints << std::endl;
                for (int joint_idx = 0; joint_idx < num_joints; ++joint_idx)
                {
                    b3JointInfo joint_info;
                    _bullet_client->getJointInfo(body_unique_id, joint_idx, &joint_info);
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
                _table_idx = body_unique_id;
            }
        }
        return ReturnCode::SUCCESS;
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
