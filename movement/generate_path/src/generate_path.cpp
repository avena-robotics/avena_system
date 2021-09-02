#include "generate_path/generate_path.hpp"

namespace generate_path
{
    GeneratePath::GeneratePath(rclcpp::Node::SharedPtr node)
        : _node(node)
    {
        auto log_level = rcutils_logging_get_logger_level(_node->get_logger().get_name());
        if (log_level == RCUTILS_LOG_SEVERITY_DEBUG)
            ompl::msg::setLogLevel(ompl::msg::LOG_DEBUG);
        else if (log_level == RCUTILS_LOG_SEVERITY_INFO)
            ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
        else if (log_level == RCUTILS_LOG_SEVERITY_WARN)
            ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
        else if (log_level == RCUTILS_LOG_SEVERITY_ERROR || log_level == RCUTILS_LOG_SEVERITY_FATAL)
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

        _initialize();
    }

    GeneratePath::~GeneratePath()
    {
        _shutdown();
    }

    void GeneratePath::_initialize()
    {
        _joint_state_sub = _node->create_subscription<sensor_msgs::msg::JointState>("arm_joint_states", 10,
                                                                                    [this](const sensor_msgs::msg::JointState::SharedPtr joint_states_msg)
                                                                                    {
                                                                                        std::lock_guard<std::mutex> lg(_current_joint_states_mtx);
                                                                                        //   RCLCPP_DEBUG_THROTTLE(_node->get_logger(), *[Generate path] get_clock(), 1000, "Received joint states [message throttles with 0.1 sec]");
                                                                                        _current_joint_states = joint_states_msg;
                                                                                    });

        if (_getParametersFromServer() != ReturnCode::SUCCESS)
            throw std::runtime_error("Cannot read parameters from server");
        
        _physics_client_handler = std::make_shared<physics_client_handler::PhysicsClientHandler>(_robot_info, _node->get_logger());
        _ik_engine = std::make_shared<inverse_kinematics::InverseKinematics>(_physics_client_handler, _robot_info, _node->get_logger());
    }

    void GeneratePath::_shutdown()
    {
        _joint_state_sub.reset();
        _physics_client_handler.reset();
        _ik_engine.reset();
    }

    GeneratedPath::SharedPtr GeneratePath::generatePath(const InputData::SharedPtr generate_path_input)
    {
        helpers::Timer timer("Generate path to pose", _node->get_logger());

        // TODO: This function has to be implemented
        _updateOctomap(generate_path_input->octomap);

        auto current_joint_states = _getCurrentJointStates();
        if (!current_joint_states)
            throw std::runtime_error("There is no joint values coming to module");

        RCLCPP_INFO(_node->get_logger(), "[Generate path] Generating pose path");
        _physics_client_handler->initializeConnection();
        if (!_physics_client_handler->isSceneValid())
            throw std::runtime_error("Scene in physics server is not valid");

        auto current_joint_values = _getJointStatesFromTopic(current_joint_states);

        PathPlanningInput path_planning_input;
        // Save start and goal state and scene info
        path_planning_input.physics_client_handler = _physics_client_handler;
        path_planning_input.state_space_size = _robot_info.nr_joints;
        path_planning_input.limits = _robot_info.soft_limits;
        path_planning_input.start_state = current_joint_values;
        
        // Check initial state validity
        RCLCPP_DEBUG(_node->get_logger(), "[Generate path] Validating initial state.");
        _validateArmConfiguration(path_planning_input.start_state);

        GeneratedPath::SharedPtr generated_path = std::make_shared<GeneratedPath>();
        generated_path->path_segments.resize(generate_path_input->movement_sequence.size());

        // Iterate over all request end effector poses, generate path for each of them
        for (size_t seq_element_id = 0; seq_element_id < generate_path_input->movement_sequence.size(); seq_element_id++)
        {
            RCLCPP_INFO(_node->get_logger(), "[Generate path] Generating path for end effector %d / %d", seq_element_id + 1, generate_path_input->movement_sequence.size());

            const auto &req_end_effector_pose = generate_path_input->movement_sequence[seq_element_id];

            // Inverse kinematics
            RCLCPP_DEBUG(_node->get_logger(), "[Generate path] Inverse kinematics calculation");
            helpers::converters::geometryToEigenAffine(req_end_effector_pose.pose, path_planning_input.goal_end_effector_pose);
            _physics_client_handler->drawCoordinateAxes(path_planning_input.goal_end_effector_pose);
            _physics_client_handler->setJointStates(path_planning_input.start_state);
            path_planning_input.goal_state = _ik_engine->computeIk(path_planning_input.goal_end_effector_pose);

            if (path_planning_input.goal_state.size() == 0)
                throw std::runtime_error("Could not find valid arm configuration for provided end effector pose");

            // TODO: Handle "path_type" and choose different planner according to type
            switch (req_end_effector_pose.path_type)
            {
            case EndEffectorPose::PATH:
                RCLCPP_INFO(_node->get_logger(), "[Generate path] Path type: PATH");
                break;
            case EndEffectorPose::LINEAR:
                RCLCPP_INFO(_node->get_logger(), "[Generate path] Path type: LINEAR");
                break;
            }

            // Path planning
            // Set joint states back to initial state before planning
            _physics_client_handler->setJointStates(path_planning_input.start_state);

            Path out_path;
            Planner planer(_node->get_logger());
            if (planer.solve(path_planning_input, out_path) != ReturnCode::SUCCESS)
                throw std::runtime_error("Error occured while planning path");

            // Set arm to last config for visualization and begging of next
            auto last_arm_config = out_path.back();
            _physics_client_handler->setJointStates(last_arm_config);

            _convertPathSegmentToTrajectoryMsg(out_path, generated_path->path_segments[seq_element_id]);

            path_planning_input.start_state = last_arm_config;
        }

        RCLCPP_INFO(_node->get_logger(), "[Generate path] Generate to pose finished");
        return generated_path;
    }

    void GeneratePath::_validateArmConfiguration(const ArmConfiguration &joint_state)
    {
        _physics_client_handler->setJointStates(joint_state);
        if (_checkJointLimits(joint_state, _robot_info.limits) != ReturnCode::SUCCESS)
            throw std::runtime_error("Invalid state. Joint states are outside of limits");

        if (_physics_client_handler->inCollision())
            throw std::runtime_error("Invalid state. Robot is in collision with scene or itself");
    }

    ReturnCode GeneratePath::_checkJointLimits(const ArmConfiguration &joint_states, const std::vector<Limits> &joint_limits)
    {
        RCLCPP_DEBUG(_node->get_logger(), "[Generate path] Check whether joint states are in limits");
        if (joint_states.size() != joint_limits.size())
        {
            RCLCPP_WARN_STREAM(_node->get_logger(), "[Generate path] Amount of joint states (" << joint_states.size() << ") is different than amount of joint limits (" << joint_limits.size() << ")");
            return ReturnCode::FAILURE;
        }
        ReturnCode error_code = ReturnCode::SUCCESS;
        for (size_t i = 0; i < joint_states.size(); ++i)
        {
            if (joint_states[i] < joint_limits[i].lower || joint_states[i] > joint_limits[i].upper)
            {
                RCLCPP_DEBUG_STREAM(_node->get_logger(), "[Generate path] Joint: " << i + 1 << ": value: " << joint_states[i] << ", limits: (" << joint_limits[i].lower << ", " << joint_limits[i].upper << ") [outside of limits]");
                error_code = ReturnCode::FAILURE;
            }
            else
                RCLCPP_DEBUG_STREAM(_node->get_logger(), "[Generate path] Joint: " << i + 1 << ": value: " << joint_states[i] << ", limits: (" << joint_limits[i].lower << ", " << joint_limits[i].upper << ")");
        }
        return error_code;
    }

    ArmConfiguration GeneratePath::_getJointStatesFromTopic(const sensor_msgs::msg::JointState::SharedPtr &joint_states)
    {
        if (joint_states->position.size() != joint_states->name.size())
            return {};

        ArmConfiguration current_joint_states;
        for (size_t i = 0; i < joint_states->position.size(); ++i)
        {
            const auto joint_name = joint_states->name[i];
            auto it = std::find(_robot_info.joint_names.begin(), _robot_info.joint_names.end(), joint_name);
            if (it != _robot_info.joint_names.end())
                current_joint_states.push_back(joint_states->position[i]);
        }

        return current_joint_states;
    }

    ReturnCode GeneratePath::_getParametersFromServer()
    {
        RCLCPP_INFO(_node->get_logger(), "[Generate path] Reading parameters from the server");

        nlohmann::json parameters = helpers::commons::getParameter("robot");
        if (parameters.empty())
            return ReturnCode::FAILURE;

        if (auto robot_info = helpers::commons::getRobotInfo())
            _robot_info = *robot_info;
        else
            return ReturnCode::FAILURE;

        // Debug display of joint limits
        std::stringstream ss;
        ss << std::endl
           << "Limits for path planning (soft limits):" << std::endl;
        for (size_t i = 0; i < _robot_info.soft_limits.size(); ++i)
            ss << "  joint " << i + 1 << ": (" << _robot_info.soft_limits[i].lower << ", " << _robot_info.soft_limits[i].upper << ")" << std::endl;

        ss << "Limits for (hard limits):" << std::endl;
        for (size_t i = 0; i < _robot_info.limits.size(); ++i)
            ss << "  joint " << i + 1 << ": (" << _robot_info.limits[i].lower << ", " << _robot_info.limits[i].upper << ")" << std::endl;
        RCLCPP_DEBUG(_node->get_logger(), "[Generate path] " + ss.str());

        RCLCPP_INFO(_node->get_logger(), "[Generate path] Parameters read successfully...");
        return ReturnCode::SUCCESS;
    }

    void GeneratePath::_convertPathSegmentToTrajectoryMsg(const std::vector<ArmConfiguration> &path, trajectory_msgs::msg::JointTrajectory &path_segment)
    {
        path_segment.points.resize(path.size());
        path_segment.joint_names = _robot_info.joint_names;
        for (size_t i = 0; i < path.size(); ++i)
        {
            path_segment.points[i].positions = path[i];
        }
    }

    sensor_msgs::msg::JointState::SharedPtr GeneratePath::_getCurrentJointStates()
    {
        // Save current state of joints to prevent data races
        sensor_msgs::msg::JointState::SharedPtr current_joint_states;
        std::lock_guard<std::mutex> lg(_current_joint_states_mtx);
        if (!_current_joint_states)
            return nullptr;
        current_joint_states = std::make_shared<sensor_msgs::msg::JointState>(*_current_joint_states);
        _current_joint_states.reset();
        return current_joint_states;
    }

    void GeneratePath::_updateOctomap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &/*octomap*/)
    {
        // TODO: Implement
    }

} // namespace generate_path
