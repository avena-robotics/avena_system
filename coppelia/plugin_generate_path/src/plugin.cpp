#include "ompl.h"
#include "stubs.h"
#include "custom_interfaces/srv/get_coppelia_handle.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

Plugin::Plugin()
{
    _parameters_read = false;
}

void Plugin::onStart()
{
    // if (!registerScriptStuff())
    //     throw std::runtime_error("script stuff initialization failed");

    _node = rclcpp::Node::make_shared("generate_path");
    helpers::commons::setLoggerLevel(_node->get_logger(), "info");

    // _watchdog = std::make_shared<helpers::Watchdog>(_node.get(), "system_monitor");

    setExtVersion("Plugin for publishing gripper states");
    setBuildDate(BUILD_DATE);

    rclcpp::QoS latching_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    _joint_state_sub = _node->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,
                                                                                [this](const sensor_msgs::msg::JointState::SharedPtr joint_states_msg)
                                                                                {
                                                                                    // RCLCPP_DEBUG_THROTTLE(_node->get_logger(), *_node->get_clock(), 500, "Received joint states [message throttles with 0.5 sec]");
                                                                                    _joint_states_data = joint_states_msg;
                                                                                });
    _pub_path = _node->create_publisher<trajectory_msgs::msg::JointTrajectory>("generated_path", latching_qos);

    _action_server_pick = rclcpp_action::create_server<custom_interfaces::action::GeneratePathPickAction>(
        _node, "generate_path_pick",
        std::bind(&Plugin::_handleGoalPick, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Plugin::_handleCancelPick, this, std::placeholders::_1),
        std::bind(&Plugin::_handleAcceptedPick, this, std::placeholders::_1));

    _action_server_place = rclcpp_action::create_server<custom_interfaces::action::GeneratePathPlaceAction>(
        _node, "generate_path_place",
        std::bind(&Plugin::_handleGoalPlace, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Plugin::_handleCancelPlace, this, std::placeholders::_1),
        std::bind(&Plugin::_handleAcceptedPlace, this, std::placeholders::_1));

    _action_server_home = rclcpp_action::create_server<custom_interfaces::action::SimpleAction>(
        _node, "generate_path_home",
        std::bind(&Plugin::_handleGoalHome, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Plugin::_handleCancelHome, this, std::placeholders::_1),
        std::bind(&Plugin::_handleAcceptedHome, this, std::placeholders::_1));

    _action_server_pose = rclcpp_action::create_server<GeneratePathPose>(
        _node, "generate_path_pose",
        std::bind(&Plugin::_handleGoalPose, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Plugin::_handleCancelPose, this, std::placeholders::_1),
        std::bind(&Plugin::_handleAcceptedPose, this, std::placeholders::_1));

    _getParametersFromServer();
    _initialize();
}

void Plugin::onInstancePass(const sim::InstancePassFlags &flags, bool first)
{
    rclcpp::spin_some(_node);
}

int Plugin::_getParametersFromServer()
{
    RCLCPP_INFO_ONCE(_node->get_logger(), "Reading parameters from the server");

    if (_parameters_read)
        return 0;

    nlohmann::json parameters = helpers::commons::getParameter("robot");
    if (parameters.empty())
        return 1;

    const std::string working_side = parameters["working_side"];
    _robot_info = helpers::commons::getRobotInfo(working_side);
    if (working_side == "right")
        _collision_robot_info = helpers::commons::getRobotInfo("left");
    else
        _collision_robot_info = helpers::commons::getRobotInfo("right");

    _parameters_read = true;
    RCLCPP_INFO(_node->get_logger(), "Parameters read successfully...");
    return 0;
}

void Plugin::_initialize() //!ok
{
    _world_rotation.clear();
    _joint_handles.clear();
    _bounds.clear();
    _frames.clear();
    _collisionPairHandles.clear();
    _joint_numbers = _robot_info.nr_joints;
    _joint_handles.resize(_joint_numbers);

    _frames = _robot_info.joint_names;

    for (size_t idx = 0; idx < _robot_info.joint_names.size(); ++idx)
    {
        std::string joint_name = _robot_info.joint_names[idx];
        _joint_handles[idx] = simGetObjectHandle(joint_name.c_str());
        _joint_names.push_back(joint_name);
    }

    // Home pose joint values
    _metric = METRICS.at(_robot_info.robot_name);
    _home_pose = HOME_JOINTS_STATE.at(_robot_info.robot_name);

    //////////////////////////////////////////////////////////////////
    // TODO: Should be extracted to separate plugin which control brain gripper
    // Gripper
    _left_finger_handle = simGetObjectHandle((_robot_info.robot_prefix + "_gripper_joint_1").c_str());
    _right_finger_handle = simGetObjectHandle((_robot_info.robot_prefix + "_gripper_joint_2").c_str());
    //////////////////////////////////////////////////////////////////

    // IK
    _ikGroup = simGetIkGroupHandle((_robot_info.robot_prefix + "_ik").c_str());
    _ikTarget = simGetObjectHandle((_robot_info.robot_prefix + "_target").c_str());
    _ikTip = simGetObjectHandle((_robot_info.robot_prefix + "_tip").c_str());

    // Collections
    const std::string robot_collection_name = _robot_info.robot_prefix + "_collection";
    _robotCollectionHandle = simGetCollectionHandle(robot_collection_name.c_str());
    const std::string collision_robot_collection_name = _collision_robot_info.robot_prefix + "_collection";
    _collisionRobotCollectionHandle = simGetCollectionHandle(collision_robot_collection_name.c_str());
    _sceneCollectionHandle = simGetCollectionHandle("scene_collection");

    _hand = simGetObjectHandle((_robot_info.robot_prefix + "_gripper_connection").c_str());
    _desk = simGetObjectHandle("Table");
    _parent_handle = simGetObjectHandle("collisions_parent");

    // _home_target = simGetObjectHandle("Home_target");

    _world_frame = simGetObjectHandle("World_frame");
    _dummyWorld_handle = simGetObjectHandle("DummyWorld");

    g_dummyWorld_handle = _dummyWorld_handle;

    std::vector<float> world_matrix(16);
    simGetObjectMatrix(_world_frame, _dummyWorld_handle, world_matrix.data());

    g_world_rotation = getRotation(world_matrix);

    // Collision pairs
    // Working robot <-> scene collision pair
    _collisionPairHandles.push_back(_robotCollectionHandle);
    _collisionPairHandles.push_back(_collisionRobotCollectionHandle);

    // Working robot <-> collsion robot collision pair
    _collisionPairHandles.push_back(_robotCollectionHandle);
    _collisionPairHandles.push_back(_sceneCollectionHandle);

    // Joints bounds
    _bounds.resize(_robot_info.nr_joints);
    for (size_t i = 0; i < _robot_info.bounds.size(); ++i)
    {
        _bounds[i].boundsLow = _robot_info.bounds[i].bounds_low;
        _bounds[i].boundsHigh = _robot_info.bounds[i].bounds_high;
    }

    // Set limits for joints in Coppelia
    for (size_t i = 0; i < _joint_handles.size(); ++i)
    {
        std::vector<float> joint_limits = {_bounds[i].boundsLow, _bounds[i].boundsHigh - _bounds[i].boundsLow};
        simSetJointInterval(_joint_handles[i], false, joint_limits.data());
    }

    // Set GLOBAL joint names
    g_joint_names = _robot_info.joint_names;

    // Set GLOBAL bounds
    g_bounds = _bounds;
}

// ___Pick action___
rclcpp_action::GoalResponse Plugin::_handleGoalPick(const rclcpp_action::GoalUUID & /*uuid*/,
                                                    std::shared_ptr<const GeneratePathPick::Goal> /*goal*/) //!ok
{
    RCLCPP_INFO(_node->get_logger(), "Goal acceped preceeding to execute goal_pick");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Plugin::_handleCancelPick(const std::shared_ptr<GoalHandleGeneratePathPick> /*goal_handle*/) //!ok
{
    RCLCPP_INFO(_node->get_logger(), "Goal Pick canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Plugin::_handleAcceptedPick(const std::shared_ptr<GoalHandleGeneratePathPick> goal_handle) //!ok
{
    RCLCPP_INFO(_node->get_logger(), "Goal Pick accepted");
    _executePick(goal_handle);
    // std::thread{std::bind(&Plugin::_executePick, this, std::placeholders::_1), goal_handle}.detach();
}

void Plugin::_executePick(const std::shared_ptr<GoalHandleGeneratePathPick> goal_handle) //!ok
{
    // rclcpp::Rate loop_rate(1);
    // const auto goal = goal_handle->get_goal();
    _target_place_data.grasp_poses.clear();
    _target_place_data.grasp_poses.push_back(goal_handle.get()->get_goal()->grasp_pose);
    constrain_value = goal_handle.get()->get_goal()->constrain_value;
    ik_trials_number = goal_handle.get()->get_goal()->ik_trials_number;
    max_final_states = goal_handle.get()->get_goal()->max_final_states;
    ompl_compare_trials = goal_handle.get()->get_goal()->ompl_compare_trials;
    min_path_points = goal_handle.get()->get_goal()->min_path_points;
    max_time = goal_handle.get()->get_goal()->max_time;
    max_simplification_time = goal_handle.get()->get_goal()->max_simplification_time;

    std::cout << "goal handle accepted with parameters " << std::endl;
    std::cout << "constrain_value" << constrain_value << std::endl;
    std::cout << "ik_trials_number" << ik_trials_number << std::endl;
    std::cout << "max_final_states" << max_final_states << std::endl;
    std::cout << "ompl_compare_trials" << ompl_compare_trials << std::endl;
    std::cout << "min_path_points" << min_path_points << std::endl;
    std::cout << "max_time" << max_time << std::endl;
    std::cout << "max_simplification_time" << max_simplification_time << std::endl;
    auto feedback = std::make_shared<GeneratePathPick::Feedback>();
    auto result = std::make_shared<GeneratePathPick::Result>();

    if (_getParametersFromServer())
    {
        RCLCPP_ERROR(_node->get_logger(), "Parameters are not read.");
        _pub_path->publish(trajectory_msgs::msg::JointTrajectory());
        goal_handle->abort(result);
        return;
    }

    if (_target_place_data.grasp_poses.empty())
    {
        RCLCPP_ERROR(_node->get_logger(), "Invalid input message. Goal failed.");
        _pub_path->publish(trajectory_msgs::msg::JointTrajectory());
        goal_handle->abort(result);
        return;
    }

    _initialize();
    _setCurrentArmState();
    if (!_generatePath(Job::PICK))
    {
        _target_place_data.grasp_poses.clear();
        goal_handle->abort(result);
        RCLCPP_INFO(_node->get_logger(), "Generate path pick aborted.");
        return;
    }

    if (rclcpp::ok())
    {
        _path_to_pick = _path_msg;
        result->path_to_pick = _path_to_pick;
        goal_handle->succeed(result);
        RCLCPP_INFO(_node->get_logger(), "Goal pick succeeded ");
    }
    RCLCPP_INFO(_node->get_logger(), "Generate pick finished");
}

// ___Place action___
rclcpp_action::GoalResponse Plugin::_handleGoalPlace(const rclcpp_action::GoalUUID & /*uuid*/,
                                                     std::shared_ptr<const GeneratePathPlace::Goal> /*goal*/)
{
    RCLCPP_INFO(_node->get_logger(), "Goal acceped preceeding to execute goal_place");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Plugin::_handleCancelPlace(const std::shared_ptr<GoalHandleGeneratePathPlace> /*goal_handle*/)
{
    RCLCPP_INFO(_node->get_logger(), "Goal Place canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Plugin::_handleAcceptedPlace(const std::shared_ptr<GoalHandleGeneratePathPlace> goal_handle) //!ok
{
    RCLCPP_INFO(_node->get_logger(), "Goal Place accepted");
    _executePlace(goal_handle);
    // std::thread{std::bind(&Plugin::_executePlace, this, std::placeholders::_1), goal_handle}.detach();
}

void Plugin::_executePlace(const std::shared_ptr<GoalHandleGeneratePathPlace> goal_handle) //!ok
{
    helpers::Timer("_executePlace", true);
    auto feedback = std::make_shared<GeneratePathPlace::Feedback>();
    auto result = std::make_shared<GeneratePathPlace::Result>();
    _target_place_data.grasp_poses.clear();
    _target_place_data.grasp_poses.push_back(goal_handle.get()->get_goal()->grasp_pose);
    _target_place_data.place_poses = goal_handle.get()->get_goal()->place_poses;
    _target_place_data.selected_item_id = goal_handle.get()->get_goal()->selected_item_id;

    constrain_value = goal_handle.get()->get_goal()->constrain_value;
    ik_trials_number = goal_handle.get()->get_goal()->ik_trials_number;
    max_final_states = goal_handle.get()->get_goal()->max_final_states;
    ompl_compare_trials = goal_handle.get()->get_goal()->ompl_compare_trials;
    min_path_points = goal_handle.get()->get_goal()->min_path_points;
    max_time = goal_handle.get()->get_goal()->max_time;
    max_simplification_time = goal_handle.get()->get_goal()->max_simplification_time;

    std::cout << "goal handle accepted with parameters " << std::endl;
    std::cout << "constrain_value " << constrain_value << std::endl;
    std::cout << "ik_trials_number" << ik_trials_number << std::endl;
    std::cout << "max_final_states" << max_final_states << std::endl;
    std::cout << "ompl_compare_trials" << ompl_compare_trials << std::endl;
    std::cout << "min_path_points" << min_path_points << std::endl;
    std::cout << "max_time" << max_time << std::endl;
    std::cout << "max_simplification_time" << max_simplification_time << std::endl;

    //------------------------------------------------------------------------------
    {
        g_selected_item_id = _target_place_data.selected_item_id;
        g_constraint_tolerance = constrain_value;

        int object_count;
        int *returned_handles = simGetObjectsInTree(_parent_handle, sim_handle_all, 2, &object_count);
        std::vector<int> objects = std::vector<int>(returned_handles, returned_handles + object_count);
        std::vector<std::string> seglist;

        for (size_t i = 0; i < objects.size(); i++)
        {
            std::stringstream ss;
            std::string segment;
            std::string name = simGetObjectName(objects[i]);
            ss << name;
            seglist.clear();
            while (std::getline(ss, segment, '_'))
                seglist.push_back(segment);
            try
            {
                int number = std::stoi(seglist[seglist.size() - 1]);
                if (number == g_selected_item_id)
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "item to grab : %s", name.c_str());
                    g_selected_item_handle = objects[i];
                }
            }
            catch (const std::exception &e)
            {
                // RCLCPP_ERROR(_node->get_logger(), "Item dont have specified name, there is lack of _number.");
                // goal_handle->abort(result);
                // return;
            }
        }
    }

    //------------------------------------------------------------------------------
    // {
    //     g_selected_item_id = _target_place_data.selected_item_id;
    //     // g_constraint_tolerance = 10;
    //     g_constraint_tolerance = constrain_value;
    //     std::string str_look_for = "_" + std::to_string(g_selected_item_id);
    //     int object_count;
    //     int *returned_handles = simGetObjectsInTree(_parent_handle, sim_handle_all, 2, &object_count);
    //     // trajectory_msgs::msg::JointTrajectoryPoint point;
    //     // point.positions.resize(_frames.size());

    //     std::vector<int> objects = std::vector<int>(returned_handles, returned_handles + object_count);
    //     for (size_t i = 0; i < objects.size(); i++)
    //     {
    //         std::string name = simGetObjectName(objects[i]);
    //         size_t res = name.find(str_look_for);
    //         if (res != std::string::npos)
    //         {
    //             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "item to grab : %s", name.c_str());
    //             g_selected_item_handle = objects[i];
    //         }
    //     }
    // }
    //------------------------------------------------------------------------------
    if (_getParametersFromServer())
    {
        RCLCPP_ERROR(_node->get_logger(), "Parameters are not read.");
        _pub_path->publish(trajectory_msgs::msg::JointTrajectory());
        goal_handle->abort(result);
        return;
    }

    if (_target_place_data.grasp_poses.empty())
    {
        RCLCPP_ERROR(_node->get_logger(), "Grasp is empty. Goal failed.");
        _pub_path->publish(trajectory_msgs::msg::JointTrajectory());
        goal_handle->abort(result);
        return;
    }

    _initialize();
    _setCurrentArmState();
    if (!_generatePath(Job::PLACE))
    {
        RCLCPP_ERROR(_node->get_logger(), "Goal Place failed.");
        goal_handle->abort(result);
        return;
    }

    if (rclcpp::ok())
    {
        _pick_to_place = _path_msg;
        result->pick_to_place = _pick_to_place;
        goal_handle->succeed(result);
        RCLCPP_INFO(_node->get_logger(), "Goal Place succeeded");
    }
    RCLCPP_INFO(_node->get_logger(), "Generate Place finished");
}

// ___Home action___
rclcpp_action::GoalResponse Plugin::_handleGoalHome(const rclcpp_action::GoalUUID & /*uuid*/,
                                                    std::shared_ptr<const GeneratePath::Goal> /*goal*/)
{
    RCLCPP_INFO(_node->get_logger(), "Goal acceped preceeding to execute goal_home");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Plugin::_handleCancelHome(const std::shared_ptr<GoalHandleGeneratePath> /*goal_handle*/)
{
    RCLCPP_INFO(_node->get_logger(), "Goal Home canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Plugin::_handleAcceptedHome(const std::shared_ptr<GoalHandleGeneratePath> goal_handle)
{
    RCLCPP_INFO(_node->get_logger(), "Goal Home accepted");
    _executeHome(goal_handle);
    // std::thread{std::bind(&Plugin::_execute_home, this, std::placeholders::_1), goal_handle}.detach();
}

void Plugin::_executeHome(const std::shared_ptr<GoalHandleGeneratePath> goal_handle)
{
    RCLCPP_INFO(_node->get_logger(), "Generating Home path");

    auto feedback = std::make_shared<GeneratePath::Feedback>();
    auto result = std::make_shared<GeneratePath::Result>();

    if (_getParametersFromServer())
    {
        RCLCPP_ERROR(_node->get_logger(), "Parameters are not read.");
        _pub_path->publish(trajectory_msgs::msg::JointTrajectory());
        goal_handle->abort(result);
        return;
    }

    _initialize();
    _setCurrentArmState();
    if (!_generatePath(Job::HOME))
    {
        RCLCPP_ERROR(_node->get_logger(), "Generate path Home aborted.");
        goal_handle->abort(result);
        return;
    }

    if (rclcpp::ok())
    {
        goal_handle->succeed(result);
        RCLCPP_INFO(_node->get_logger(), "Goal Home succeeded");
    }
    RCLCPP_INFO(_node->get_logger(), "Generate Home finished");
}

// ___Pose action___
rclcpp_action::GoalResponse Plugin::_handleGoalPose(const rclcpp_action::GoalUUID & /*uuid*/,
                                                    std::shared_ptr<const GeneratePathPose::Goal> /*goal*/)
{
    RCLCPP_INFO(_node->get_logger(), "Goal acceped preceeding to execute goal_pose");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Plugin::_handleCancelPose(const std::shared_ptr<GoalHandleGeneratePathPose> /*goal_handle*/)
{
    RCLCPP_INFO(_node->get_logger(), "Goal to pose canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Plugin::_handleAcceptedPose(const std::shared_ptr<GoalHandleGeneratePathPose> goal_handle)
{
    RCLCPP_INFO(_node->get_logger(), "Goal to pose accepted");
    _executePose(goal_handle);
}

void Plugin::_executePose(const std::shared_ptr<GoalHandleGeneratePathPose> goal_handle)
{
    RCLCPP_INFO(_node->get_logger(), "Generating pose path");
    auto result = std::make_shared<GeneratePathPose::Result>();

    if (_getParametersFromServer())
    {
        RCLCPP_ERROR(_node->get_logger(), "Parameters are not read.");
        _pub_path->publish(trajectory_msgs::msg::JointTrajectory());
        goal_handle->abort(result);
        return;
    }

    _target_pose_data = goal_handle->get_goal()->end_effector_pose;

    _initialize();
    _setCurrentArmState();
    if (!_generatePath(Job::POSE))
    {
        RCLCPP_ERROR(_node->get_logger(), "Generate path to pose aborted.");
        goal_handle->abort(result);
        return;
    }

    if (rclcpp::ok())
    {
        goal_handle->succeed(result);
        RCLCPP_INFO(_node->get_logger(), "Goal to pose succeeded");
    }
    RCLCPP_INFO(_node->get_logger(), "Generate to pose finished");
}

void Plugin::_getPlaceTargetCallback(const custom_interfaces::msg::Place::SharedPtr _target_place_msg)
{
    g_selected_item_id = _target_place_data.selected_item_id;
    g_constraint_tolerance = 10;
    std::string str_look_for = "_" + std::to_string(g_selected_item_id);
    int object_count;
    int *returned_handles = simGetObjectsInTree(_parent_handle, sim_handle_all, 2, &object_count);
    // trajectory_msgs::msg::JointTrajectoryPoint point;
    // point.positions.resize(_frames.size());

    std::vector<int> objects = std::vector<int>(returned_handles, returned_handles + object_count);
    for (size_t i = 0; i < objects.size(); i++)
    {
        std::string name = simGetObjectName(objects[i]);
        size_t res = name.find(str_look_for);
        if (res != std::string::npos)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "item to grab : %s", name.c_str());
            g_selected_item_handle = objects[i];
        }
    }
}

// void Plugin::_broadcastPath()
// {
//     _pub_path->publish(_path_msg);
// }

// bool Plugin::_check_4_5_links_collision_distance(float treshold)
// {
//     std::vector<float> distance_data(_joint_numbers);
//     int wall_handle = simGetObjectHandle("Wall_blocking_move");
//     int link_4_handle = simGetObjectHandle("Franka_link4_ghost");
//     int link_5_handle = simGetObjectHandle("Franka_link5_ghost");

//     int result = simCheckDistance(wall_handle, link_4_handle, treshold, distance_data.data());
//     if (result != 0)
//         return false;

//     result = simCheckDistance(wall_handle, link_5_handle, treshold, distance_data.data());
//     if (result != 0)
//         return false;

//     return true;
// }

std::vector<float> Plugin::_getShiftAlongZAxisMatrix(std::vector<float> matrix)
{
    std::vector<float> shifted_matrix(matrix);
    shifted_matrix[11] += _z_shift;

    return shifted_matrix;
}

std::vector<float> Plugin::_getShiftAlongGripperAxisMatrix(std::vector<float> matrix)
{
    std::vector<float> shifted_matrix(matrix);
    shifted_matrix[3] += matrix[0] * _x_shift;
    shifted_matrix[7] += matrix[4] * _x_shift;
    shifted_matrix[11] += matrix[8] * _x_shift;

    return shifted_matrix;
}

TargetMatrix Plugin::_setCartesianPlace()
{
    TargetMatrix place_matrix;
    double offset_place = 0.005;

    for (int i = 0; i < int(_target_place_data.place_poses[0].poses.size()); i++)
    {
        const float place_pose[3] = {float(_target_place_data.place_poses[0].poses[i].position.x),
                                     float(_target_place_data.place_poses[0].poses[i].position.y),
                                     float(_target_place_data.place_poses[0].poses[i].position.z + offset_place)};

        const float place_quaternion[4] = {float(_target_place_data.place_poses[0].poses[i].orientation.x),
                                           float(_target_place_data.place_poses[0].poses[i].orientation.y),
                                           float(_target_place_data.place_poses[0].poses[i].orientation.z),
                                           float(_target_place_data.place_poses[0].poses[i].orientation.w)};

        std::vector<float> temp_matrix;
        temp_matrix.resize(16);
        _setTarget(place_pose, place_quaternion, "Place_target");
        simGetObjectMatrix(_handle, _frame_handle, temp_matrix.data());
        place_matrix.final_target.push_back(temp_matrix);

        // calculate shifted matrix for pre place pose
        std::vector<float> pre_temp_matrix = _getShiftAlongZAxisMatrix(temp_matrix);
        place_matrix.pre_target.push_back(pre_temp_matrix);
    }

    const float post_grasp_pose[3] = {float(_target_place_data.grasp_poses[0].pregrasp_pose.position.x),
                                      float(_target_place_data.grasp_poses[0].pregrasp_pose.position.y),
                                      float(_target_place_data.grasp_poses[0].pregrasp_pose.position.z)};

    const float post_grasp_quaternion[4] = {float(_target_place_data.grasp_poses[0].pregrasp_pose.orientation.x),
                                            float(_target_place_data.grasp_poses[0].pregrasp_pose.orientation.y),
                                            float(_target_place_data.grasp_poses[0].pregrasp_pose.orientation.z),
                                            float(_target_place_data.grasp_poses[0].pregrasp_pose.orientation.w)};

    std::vector<float> temp_matrix3;
    temp_matrix3.resize(16);
    _setTarget(post_grasp_pose, post_grasp_quaternion, "Pregrasp_target");
    simGetObjectMatrix(_handle, _frame_handle, temp_matrix3.data());
    place_matrix.post_start.push_back(temp_matrix3);

    return place_matrix;
}

TargetMatrix Plugin::_setCartesianPick() //!ok
{
    TargetMatrix grasp_matrix;

    for (size_t i = 0; i < _target_place_data.grasp_poses.size(); i++)
    {
        const float grasp_pose[3] = {float(_target_place_data.grasp_poses[0].grasp_pose.position.x),
                                     float(_target_place_data.grasp_poses[0].grasp_pose.position.y),
                                     float(_target_place_data.grasp_poses[0].grasp_pose.position.z)};

        const float grasp_quaternion[4] = {float(_target_place_data.grasp_poses[0].grasp_pose.orientation.x),
                                           float(_target_place_data.grasp_poses[0].grasp_pose.orientation.y),
                                           float(_target_place_data.grasp_poses[0].grasp_pose.orientation.z),
                                           float(_target_place_data.grasp_poses[0].grasp_pose.orientation.w)};

        const float pregrasp_pose[3] = {float(_target_place_data.grasp_poses[0].pregrasp_pose.position.x),
                                        float(_target_place_data.grasp_poses[0].pregrasp_pose.position.y),
                                        float(_target_place_data.grasp_poses[0].pregrasp_pose.position.z)};

        const float pregrasp_quaternion[4] = {float(_target_place_data.grasp_poses[0].pregrasp_pose.orientation.x),
                                              float(_target_place_data.grasp_poses[0].pregrasp_pose.orientation.y),
                                              float(_target_place_data.grasp_poses[0].pregrasp_pose.orientation.z),
                                              float(_target_place_data.grasp_poses[0].pregrasp_pose.orientation.w)};

        std::vector<float> temp_matrix;
        temp_matrix.resize(16);
        _setTarget(grasp_pose, grasp_quaternion, "Grasp_target");
        simGetObjectMatrix(_handle, _frame_handle, temp_matrix.data());
        grasp_matrix.final_target.push_back(temp_matrix);

        std::vector<float> temp_matrix2;
        temp_matrix2.resize(16);
        _setTarget(pregrasp_pose, pregrasp_quaternion, "Pregrasp_target");
        simGetObjectMatrix(_handle, _frame_handle, temp_matrix2.data());
        grasp_matrix.pre_target.push_back(temp_matrix2);
    }
    return grasp_matrix;
}

TargetMatrix Plugin::_setCartesianPose()
{
    TargetMatrix grasp_matrix;
    const float pose[3] = {
        static_cast<float>(_target_pose_data.position.x),
        static_cast<float>(_target_pose_data.position.y),
        static_cast<float>(_target_pose_data.position.z)};
    const float quaternion[4] = {
        static_cast<float>(_target_pose_data.orientation.x),
        static_cast<float>(_target_pose_data.orientation.y),
        static_cast<float>(_target_pose_data.orientation.z),
        static_cast<float>(_target_pose_data.orientation.w)};

    std::vector<float> temp_matrix;
    temp_matrix.resize(16);
    _setTarget(pose, quaternion, "Pose_target");
    simGetObjectMatrix(_handle, _frame_handle, temp_matrix.data());
    grasp_matrix.final_target.push_back(temp_matrix);

    return grasp_matrix;
}

std::vector<std::vector<float>> Plugin::_findCollisionFreeArmStates(TargetMatrix poses, bool no_collision) //!ok
{
    helpers::Timer czas("_findCollisionFreeArmStates", no_collision);
    std::vector<float> current_arm_state = _getArmState();
    std::vector<std::vector<float>> valid_arm_states;
    std::vector<float> distance_to_arm_state;
    std::vector<std::vector<float>> target_arm_states;

    int all_valid_trial = 0;
    for (int state_number = 0; state_number < static_cast<int>(poses.pre_target.size()); state_number++)
    {
        // RCLCPP_INFO(_node->get_logger(), "Searching for %d final pose...", state_number + 1);

        int valid_trials = 0;

        for (int trial_number = 0; trial_number < 5; trial_number++)
        {
            std::vector<float> trial_goal_state;
            trial_goal_state.resize(_joint_handles.size());

            simSetObjectMatrix(_ikTarget, _dummyWorld_handle, poses.pre_target[state_number].data());

            int result = simGetConfigForTipPose(_ikGroup, _joint_handles.size(), _joint_handles.data(), 0.35, 200,
                                                trial_goal_state.data(), nullptr, _collisionPairHandles.size() / 2, _collisionPairHandles.data(),
                                                nullptr, nullptr, nullptr, nullptr);

            if (result == 1)
            {
                float distance = _getDistanceBetweenStates(current_arm_state, trial_goal_state);
                bool are_states_similar = false;
                for (int state = 0; state < static_cast<int>(distance_to_arm_state.size()); state++)
                {
                    if (abs(distance_to_arm_state[state] - distance) < 0.001) // TODO fix hardcoded value
                    {
                        are_states_similar = true;
                        for (int joint = 0; joint < _joint_numbers; joint++)
                        {
                            if (abs(valid_arm_states[state][joint] - trial_goal_state[joint]) > 0.01)
                            {
                                are_states_similar = false;
                                break;
                            }
                        }
                    }

                    if (are_states_similar)
                        break;
                }
                if (!are_states_similar)
                {
                    _setArmState(trial_goal_state);
                    // check collsion with wall
                    // bool wall_collision = _check_4_5_links_collision_distance(0.11); // TODO fix hardcoded value
                    // if (wall_collision == false)
                    // {
                    //     // RCLCPP_WARN(_node->get_logger(), "Wall collision pose!");
                    //     continue;
                    // }
                    // if not in collision go ahead

                    // check now if it is possible to create IK path from pre to final pose
                    simSetObjectMatrix(_ikTarget, _dummyWorld_handle, poses.final_target[state_number].data());
                    float *result = nullptr;
                    if (no_collision)
                        result = simGenerateIkPath(_ikGroup, _joint_handles.size(), _joint_handles.data(), 40, 0,
                                                   nullptr, nullptr, nullptr);
                    else
                        result = simGenerateIkPath(_ikGroup, _joint_handles.size(), _joint_handles.data(), 40, _collisionPairHandles.size() / 2,
                                                   _collisionPairHandles.data(), nullptr, nullptr);

                    // if everything is allright add state as a good candidate
                    if (result != nullptr)
                    {
                        valid_arm_states.push_back(trial_goal_state);
                        distance_to_arm_state.push_back(distance);
                        valid_trials++;
                        // RCLCPP_INFO(_node->get_logger(), "Find %d valid pose", valid_trials);
                    }
                }
            }

            if (valid_trials >= _max_final_configs)
                break;
        }
        all_valid_trial += valid_trials;
    }

    RCLCPP_INFO(_node->get_logger(), "Find %d valid poses for %d place targets", all_valid_trial, poses.pre_target.size());

    // restore position
    _setArmState(current_arm_state);

    return valid_arm_states;
}

std::vector<std::vector<float>> Plugin::_findCollisionFreeArmStates(std::vector<std::vector<float>> matrix_poses) //! Do usuniecia, albo i nie
{
    std::vector<float> current_arm_state = _getArmState();
    std::vector<std::vector<float>> valid_arm_states;
    std::vector<float> distance_to_arm_state;
    std::vector<std::vector<float>> target_arm_states;
    for (int state_number = 0; state_number < static_cast<int>(matrix_poses.size()); state_number++)
    {
        RCLCPP_INFO(_node->get_logger(), "Searching for %d final pose...", state_number + 1);
        int valid_trials = 0;

        simSetObjectMatrix(_ikTarget, _dummyWorld_handle, matrix_poses[state_number].data());

        for (int trial_number = 0; trial_number < 10; trial_number++)
        {
            std::vector<float> trial_goal_state;
            trial_goal_state.resize(_joint_handles.size());

            int result = simGetConfigForTipPose(_ikGroup, _joint_handles.size(), _joint_handles.data(), 0.35, 200,
                                                trial_goal_state.data(), nullptr, _collisionPairHandles.size() / 2, _collisionPairHandles.data(),
                                                nullptr, nullptr, nullptr, nullptr);

            if (result == 1)
            {
                float distance = _getDistanceBetweenStates(current_arm_state, trial_goal_state);
                bool are_states_similar = false;
                for (int state = 0; state < static_cast<int>(distance_to_arm_state.size()); state++)
                {
                    if (abs(distance_to_arm_state[state] - distance) < 0.001) // TODO fix hardcoded value
                    {
                        are_states_similar = true;
                        for (int joint = 0; joint < _joint_numbers; joint++)
                        {
                            if (abs(valid_arm_states[state][joint] - trial_goal_state[joint]) > 0.01)
                            {
                                are_states_similar = false;
                                break;
                            }
                        }
                    }

                    if (are_states_similar)
                        break;
                }
                if (!are_states_similar)
                {
                    valid_arm_states.push_back(trial_goal_state);
                    distance_to_arm_state.push_back(distance);
                    valid_trials++;
                    RCLCPP_INFO(_node->get_logger(), "Find %d valid pose", valid_trials);
                }
            }

            if (valid_trials >= _max_final_configs)
                break;
        }
    }
    return valid_arm_states;
}

void Plugin::_setTarget(const float pose[3], const float quaternion[4], const std::string name)
{
    _handle = simGetObjectHandle(name.c_str());
    _frame_handle = simGetObjectHandle("DummyWorld");

    simSetObjectPosition(_handle, _frame_handle, pose);
    simSetObjectQuaternion(_handle, _frame_handle, quaternion);
}

void Plugin::_setCurrentArmState() //!ok
{
    int joint_index = 0;

    // if (_joint_states_data == nullptr)
    //     _joint_states_data = std::make_shared<float> _home_pose

    if (_joint_states_data->position.size() != 0 && _joint_handles.size() != 0)
    {
        for (auto position : _joint_states_data->position)
            simSetJointPosition(_joint_handles[joint_index++], position);
    }
}

void Plugin::_setArmState(std::vector<float> state) //!ok
{
    for (int i = 0; i < _joint_numbers; i++)
        simSetJointPosition(_joint_handles[i], state[i]);
}

void Plugin::_savePickState()
{
    _pick_state = _getArmState();
}

std::vector<float> Plugin::_generateIkPath(std::vector<std::vector<float>> matrix, const size_t &nr_path_points) //!ok
{
    std::vector<float> current_state = _getArmState();
    _setArmState(current_state);

    for (size_t i = 0; i < matrix.size(); i++)
    {
        simSetObjectMatrix(_ikTarget, _dummyWorld_handle, matrix[i].data());

        auto result = simGenerateIkPath(_ikGroup, _joint_handles.size(), _joint_handles.data(), nr_path_points, _collisionPairHandles.size() / 2,
                                        _collisionPairHandles.data(), nullptr, nullptr);

        if (result == nullptr)
        {
            // _setArmState(current_state);
            // return std::vector<float>();
            continue;
        }
        else
        {
            _setArmState(current_state);
            std::vector<float> ik_path = std::vector<float>(result, result + _joint_numbers * nr_path_points);
            simReleaseBuffer(reinterpret_cast<const char *>(result));
            return ik_path;
        }
    }
    _setArmState(current_state);

    return std::vector<float>();
}

std::vector<float> Plugin::_freeCollisionIkPath(std::vector<std::vector<float>> matrix) //!ok
{
    std::vector<float> current_state = _getArmState();
    _setArmState(current_state);

    for (size_t i = 0; i < matrix.size(); i++)
    {
        simSetObjectMatrix(_ikTarget, _dummyWorld_handle, matrix[i].data());

        auto result =
            simGenerateIkPath(_ikGroup, _joint_handles.size(), _joint_handles.data(), 40, 0, nullptr, nullptr, nullptr);

        if (result == nullptr)
        {
            // _setArmState(current_state);
            // return std::vector<float>();
            continue;
        }
        else
        {
            _setArmState(current_state);
            std::vector<float> ik_path = std::vector<float>(result, result + _joint_numbers * 40);
            return ik_path;
        }
    }
    _setArmState(current_state);

    return std::vector<float>();
}

void Plugin::_setLastPathConfig(std::vector<float> path) //!ok
{
    std::vector<float> last_state(_joint_numbers);

    for (int i = 0; i < _joint_numbers; i++)
        last_state[i] = path[path.size() - _joint_numbers + i];

    _setArmState(last_state);
}

void Plugin::_detachItem() //!ok
{
    //------------------------------------------------------------
    if (g_item_to_detach_handle)
    {
        RCLCPP_INFO(_node->get_logger(), "DETACH OBJECT");

        int result = simSetObjectParent(g_item_to_detach_handle, _parent_handle, true);
        if (result == -1)
        {
            RCLCPP_ERROR(_node->get_logger(), "Object detach failed");
            // do abort
        }
        _openGripper();

        // restore object
        simSetObjectMatrix(g_item_to_detach_handle, _dummyWorld_handle, g_item_matrix.data());
    }
}

bool Plugin::_generatePath(Job job) //!ok
{
    std::vector<std::vector<float>> valid_arm_states;
    std::vector<float> distance_to_arm_state;
    std::vector<std::vector<float>> target_arm_states;
    trajectory_msgs::msg::JointTrajectory calculated_path;
    std::vector<float> current_arm_state;

    // Set initial arm states
    _setCurrentArmState();
    current_arm_state = _getArmState();

    for (size_t i = 0; i < current_arm_state.size(); i++)
    {
        auto error = std::abs(_joint_states_data->position[i] - current_arm_state[i]);
        RCLCPP_DEBUG_STREAM(_node->get_logger(), "Joint number: " << i + 1 << ", error: " << error << " rad");
        if (error >= JOINT_STATE_DIFF)
        {
            RCLCPP_ERROR(_node->get_logger(), "Cannot set initial state for path generation. Exiting...");
            return false;
        }
    }

    std::vector<float> path_to_execute;

    // TODO make exceptions and encapsulate code

    TargetMatrix target_matrix;
    if (job == Job::PICK)
    {
        // set pregrasp and grasp target
        target_matrix = _setCartesianPick();

        //------------------------------------------------------------
        {
            RCLCPP_INFO(_node->get_logger(), "DO PRE GRASP");

            // find valid arms state to gregrasp pose
            // std::vector<std::vector<float>> goal_states = _findCollisionFreeArmStates(target_matrix.pre_target);
            std::vector<std::vector<float>> goal_states = _findCollisionFreeArmStates(target_matrix, false);
            if (goal_states.size() == 0)
            {
                RCLCPP_ERROR(_node->get_logger(), "Cannot find any valid final arm state. Wrong pregrasp pose. "
                                                  "Aborting");
                return false;
            }

            // try to find path with OMPL
            std::vector<float> path = _findPath(goal_states, job);
            if (static_cast<int>(path.size()) != _min_path_points * _joint_numbers)
            {
                RCLCPP_ERROR(_node->get_logger(), "Cannot Find path. Length is not correct. Aborting");
                return false;
            }

            // add path to execute path
            path_to_execute.insert(path_to_execute.end(), path.begin(), path.end());

            // move ghost to final pose
            _setLastPathConfig(path);
        }

        //------------------------------------------------------------
        {
            RCLCPP_INFO(_node->get_logger(), "DO GRASP");

            // do IK path from pregrasp to grasp poses
            std::vector<float> ik_path = _generateIkPath(target_matrix.final_target, 40);

            if (static_cast<int>(ik_path.size()) == 0)
            {
                RCLCPP_ERROR(_node->get_logger(), "Cannot generate final IK path from pre pose to final pose.");
                return false;
            }

            // add path to execute path
            path_to_execute.insert(path_to_execute.end(), ik_path.begin(), ik_path.end());

            // move ghost to final pose
            _setLastPathConfig(ik_path);

            // save pick position
            _savePickState();
        }
    }
    else if (job == Job::PLACE)
    {
        target_matrix = _setCartesianPlace();

        g_item_to_detach_handle = 0;

        // restore robot pose to pick position
        _setArmState(_pick_state);

        //------------------------------------------------------------
        {
            RCLCPP_INFO(_node->get_logger(), "ATTACH OBJECT");
            int result = simSetObjectParent(g_selected_item_handle, _hand, true);
            if (result == -1)
            {
                RCLCPP_ERROR(_node->get_logger(), "Object attach failed");
                // do abort
            }
            else
            {
                simGetObjectMatrix(g_selected_item_handle, _dummyWorld_handle, g_item_matrix.data());
                g_item_to_detach_handle = g_selected_item_handle;
            }
        }

        //------------------------------------------------------------
        {
            RCLCPP_INFO(_node->get_logger(), "DO POST GRASP");

            // do IK path from grasp to pregrasp poses without collision
            std::vector<float> ik_path = _freeCollisionIkPath(target_matrix.post_start);

            if (static_cast<int>(ik_path.size()) == 0)
            {
                RCLCPP_ERROR(_node->get_logger(), "Cannot generate final IK path from pre pose to final pose.");
                _detachItem();
                return false;
            }

            // add path to execute path
            path_to_execute.insert(path_to_execute.end(), ik_path.begin(), ik_path.end());

            // move ghost to final pose
            _setLastPathConfig(ik_path);
        }

        //------------------------------------------------------------
        {
            RCLCPP_INFO(_node->get_logger(), "SAVE CONSTRAINT");

            std::vector<float> item_matrix(16);
            simGetObjectMatrix(g_selected_item_handle, _dummyWorld_handle, item_matrix.data());
            g_item_rotation = getRotation(item_matrix);
            g_item_transform = g_item_rotation.inverse();
        }

        //------------------------------------------------------------
        {
            RCLCPP_INFO(_node->get_logger(), "DO PRE PLACE");

            std::vector<std::vector<float>> goal_states = _findCollisionFreeArmStates(target_matrix, true);

            if (goal_states.size() == 0)
            {
                RCLCPP_ERROR(_node->get_logger(), "Cannot find any valid final arm state. Target place is occupied "
                                                  "or "
                                                  "position is unachievable. Aborting");
                _detachItem();
                return false;
            }

            std::vector<float> path = _findPath(goal_states, job);

            if (static_cast<int>(path.size()) != _min_path_points * _joint_numbers)
            {
                RCLCPP_ERROR(_node->get_logger(), "Path length is not correct. Aborting");
                _detachItem();
                return false;
            }

            // add path to execute path
            path_to_execute.insert(path_to_execute.end(), path.begin(), path.end());

            // move ghost to final pose
            _setLastPathConfig(path);
        }

        //------------------------------------------------------------
        {
            RCLCPP_INFO(_node->get_logger(), "DO PLACE");

            // do IK path from grasp to pregrasp poses without collision
            std::vector<float> ik_path = _freeCollisionIkPath(target_matrix.final_target);

            if (static_cast<int>(ik_path.size()) == 0)
            {
                RCLCPP_ERROR(_node->get_logger(), "Cannot generate final IK path from pre pose to final pose.");
                _detachItem();
                return false;
            }

            // add path to execute path
            path_to_execute.insert(path_to_execute.end(), ik_path.begin(), ik_path.end());

            // move ghost to final pose
            _setLastPathConfig(ik_path);

            _detachItem();
        }
    }
    else if (job == Job::HOME)
    {
        //------------------------------------------------------------
        // {
        //     // try to go back to avoid gripper collision
        //     RCLCPP_INFO(_node->get_logger(), "DO PRE HOME IK");

        //     _setCurrentArmState();

        //     // set the pose to go back
        //     std::vector<float> current_pose_matrix(16);
        //     simGetObjectMatrix(_ikTip, _dummyWorld_handle, current_pose_matrix.data());
        //     std::vector<std::vector<float>> shited_pose_matrix;
        //     shited_pose_matrix.push_back(_getShiftAlongGripperAxisMatrix(current_pose_matrix));

        //     simSetObjectMatrix(_home_target, _dummyWorld_handle, shited_pose_matrix[0].data());

        //     // do IK path from current pose to a little back
        //     std::vector<float> ik_path = _generateIkPath(shited_pose_matrix);

        //     if (static_cast<int>(ik_path.size()) == 0)
        //     {
        //         RCLCPP_WARN(_node->get_logger(), "Cannot generate IK path for safety HOME move begin.");
        //         // return false;
        //     }
        //     else
        //     {
        //         // add path to execute path
        //         path_to_execute.insert(path_to_execute.end(), ik_path.begin(), ik_path.end());

        //         // move ghost to final pose
        //         _setLastPathConfig(ik_path);
        //     }
        // }

        //------------------------------------------------------------
        {
            RCLCPP_INFO(_node->get_logger(), "DO HOME");

            std::vector<ArmConfiguration> goal_states{_home_pose};
            if (goal_states.size() == 0)
            {
                RCLCPP_ERROR(_node->get_logger(), "Cannot find any valid final arm state. Aborting");
                return false;
            }

            std::vector<float> path = _findPath(goal_states, job);

            if (static_cast<int>(path.size()) != _min_path_points * _joint_numbers)
            {
                RCLCPP_ERROR(_node->get_logger(), "Path length is not correct. Aborting");
                return false;
            }

            // add path to execute path
            path_to_execute.insert(path_to_execute.end(), path.begin(), path.end());

            // move ghost to final pose
            _setLastPathConfig(path);
        }
    }
    else if (job == Job::POSE)
    {
        target_matrix = _setCartesianPose();

        RCLCPP_INFO(_node->get_logger(), "DO GO TO POSE");
        
        // // Simple linear interpolation
        // std::vector<float> ik_path = _generateIkPath(target_matrix.final_target, 150);

        // if (ik_path.size() > 0)
        // {
        //     // add path to execute path
        //     path_to_execute.insert(path_to_execute.end(), ik_path.begin(), ik_path.end());

        //     // move ghost to final pose
        //     _setLastPathConfig(ik_path);
        // }
        // else
        // {
        //     RCLCPP_WARN(_node->get_logger(), "Cannot generate path in straight line in Cartesian space. Trying to generate path with OMPL...");
            std::vector<ArmConfiguration> goal_states = _findCollisionFreeArmStates(target_matrix.final_target);

            if (goal_states.size() == 0)
            {
                RCLCPP_ERROR(_node->get_logger(), "Cannot find any valid final arm state. Aborting");
                return false;
            }

            std::vector<float> path = _findPath(goal_states, job);

            if (static_cast<int>(path.size()) != _min_path_points * _joint_numbers)
            {
                RCLCPP_ERROR(_node->get_logger(), "Path length is not correct. Aborting");
                return false;
            }

            // add path to execute path
            path_to_execute.insert(path_to_execute.end(), path.begin(), path.end());

            // move ghost to final pose
            _setLastPathConfig(path);
        // }
    }

    // restore robot pose
    _setArmState(current_arm_state);

    //------------------------------------------------------------
    // create Joint Trajectory Message and publish
    calculated_path = _createPathData(path_to_execute);

    rclcpp::Time now = _node->now();
    calculated_path.header.stamp = now;
    _path_msg = calculated_path;
    _pub_path->publish(_path_msg);

    return true;
}

void Plugin::_openGripper() //!ok
{
    simSetJointPosition(_left_finger_handle, 0.04);
    simSetJointPosition(_right_finger_handle, 0.04);
}

trajectory_msgs::msg::JointTrajectory Plugin::_createPathData(const std::vector<float> path)
{
    trajectory_msgs::msg::JointTrajectory path_data;
    path_data.joint_names = _frames;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(_frames.size());

    for (size_t i = 0; i < path.size() / _joint_numbers; i++)
    {
        for (size_t j = 0; j < _frames.size(); j++)
            point.positions[j] = path[i * _joint_numbers + j];

        path_data.points.push_back(point);
    }
    return path_data;
}

float Plugin::_getDistanceBetweenStates(std::vector<float> arm_state1, std::vector<float> arm_state2) //!ok
{
    float distance = 0;
    for (size_t i = 0; i < _joint_handles.size(); i++)
        distance += pow((arm_state1[i] - arm_state2[i]) * _metric[i], 2);
    return sqrt(distance);
}

float Plugin::_getPathLength(std::vector<float> path)
{
    float path_length = 0;

    for (size_t i = 0; i < (path.size() - 1) / _joint_numbers; i++)
    {
        std::vector<float> segment_one;
        std::vector<float> segment_two;
        for (int j = 0; j < _joint_numbers; j++)
        {
            segment_one.push_back(path[i] * _joint_numbers + j);
            segment_two.push_back(path[i + 1] * _joint_numbers + j);
        }

        path_length += _getDistanceBetweenStates(segment_one, segment_two);
    }

    return path_length;
}

std::vector<float> Plugin::_findPath(std::vector<std::vector<float>> goal_states, Job job) //!ok
{
    std::vector<float> if_error_appear_return_zero;
    if_error_appear_return_zero.push_back(0);
    // create task
    TaskDef *task = new TaskDef();
    task->header.destroyAfterSimulationStop = simGetSimulationState() != sim_simulation_stopped;

    task->header.handle = _nextTaskHandle++;
    task->header.name = "task_" + std::to_string(_nextTaskHandle);
    task->goal.type = TaskDef::Goal::STATE;

    if (job == Job::PLACE)
        task->stateValidation.type = TaskDef::StateValidation::CLLBACK;
    else
        task->stateValidation.type = TaskDef::StateValidation::DEFAULT;

    task->stateValidityCheckingResolution = 0.01f; // 1% of state space's extent
    task->validStateSampling.type = TaskDef::ValidStateSampling::DEFAULT;
    task->projectionEvaluation.type = TaskDef::ProjectionEvaluation::DEFAULT;
    // task->algorithm = sim_ompl_algorithm_RRTConnect;
    // task->algorithm = sim_ompl_algorithm_LBKPIECE1;
    task->algorithm = sim_ompl_algorithm_RRT;
    std::cout << "Algorithm used: " << task->algorithm << std::endl;

    task->verboseLevel = 0;

    //_createStateSpace
    std::vector<int> stateSpaceHandles;

    for (size_t i = 0; i < _joint_handles.size(); i++)
        stateSpaceHandles.push_back(_createStateSpace(i));

    //_setStateSpace
    bool valid_statespace_handles = true;

    for (size_t i = 0; i < stateSpaceHandles.size(); i++)
    {
        int stateSpaceHandle = stateSpaceHandles[i];

        if (_statespaces.find(stateSpaceHandle) == _statespaces.end())
        {
            valid_statespace_handles = false;
            break;
        }
    }

    if (!valid_statespace_handles)
    {
        RCLCPP_INFO(_node->get_logger(), "Invalid state space handle.");
        return if_error_appear_return_zero;
    }

    task->stateSpaces.clear();
    task->dim = 0;

    for (size_t i = 0; i < stateSpaceHandles.size(); i++)
    {
        task->stateSpaces.push_back(stateSpaceHandles[i]);
        task->dim += 1;
    }

    // setCollisionPairs
    task->collisionPairHandles.clear();
    for (auto &collection_pair : _collisionPairHandles)
        task->collisionPairHandles.push_back(collection_pair);

    // setStartState
    task->startState.clear();
    task->startState = _getArmState(); // check if works good
    // task->startState = _joint_states_data; // check if works good

    // setGoalStates
    for (size_t i = 0; i < goal_states.size(); i++)
    {
        std::vector<float> goal_state;
        for (size_t j = 0; j < _joint_handles.size(); j++)
            goal_state.push_back(goal_states[i][j]);

        task->goal.states.push_back(goal_state);
    }

    task->stateSpacePtr = ob::StateSpacePtr(new StateSpace(_statespaces, task));                        //! A shared pointer wrapper for ompl::base::StateSpace.
    task->spaceInformationPtr = ob::SpaceInformationPtr(new ob::SpaceInformation(task->stateSpacePtr)); //! A shared pointer wrapper for ompl::base::SpaceInformation.
    task->projectionEvaluatorPtr =
        ob::ProjectionEvaluatorPtr(new ProjectionEvaluator(_statespaces, task->stateSpacePtr, task));
    task->stateSpacePtr->registerDefaultProjection(task->projectionEvaluatorPtr);
    task->problemDefinitionPtr = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(task->spaceInformationPtr));

    task->spaceInformationPtr->setStateValidityChecker(
        ob::StateValidityCheckerPtr(new StateValidityChecker(task->spaceInformationPtr, task)));
    task->spaceInformationPtr->setStateValidityCheckingResolution(task->stateValidityCheckingResolution);
    task->spaceInformationPtr->setValidStateSamplerAllocator(
        std::bind(allocValidStateSampler, std::placeholders::_1, task));

    ob::ScopedState<> startState(task->stateSpacePtr);
    // validateStateSize(task, task->startState, "Start state");
    for (size_t i = 0; i < task->startState.size(); i++)
        startState[i] = task->startState[i];

    // check if start state is valid
    bool valid = task->spaceInformationPtr->isValid(startState.get());

    if (valid == false)
    {
        RCLCPP_ERROR(_node->get_logger(), "Start state is invalid! Probably robot is in collision with octomap or other item.");
        return if_error_appear_return_zero;
    }
    else
    {
        task->problemDefinitionPtr->addStartState(startState);
        ob::GoalPtr goal;
        if (task->goal.type == TaskDef::Goal::STATE)
        {
            // for (size_t i = 0; i < task->goal.states.size(); i++)
            // validateStateSize(task, task->goal.states[i], "Goal state");

            if (task->goal.states.size() >= 1)
            {
                goal = ob::GoalPtr(new ob::GoalStates(task->spaceInformationPtr));
                for (size_t j = 0; j < task->goal.states.size(); j++)
                {
                    ob::ScopedState<> goalState(task->stateSpacePtr);
                    for (size_t i = 0; i < task->goal.states[j].size(); i++)
                        goalState[i] = task->goal.states[j][i];
                    goal->as<ob::GoalStates>()->addState(goalState);
                }
            }
            else
            {
                RCLCPP_INFO(_node->get_logger(), "No goal state specified.");
                return if_error_appear_return_zero;
            }

            // std::cout << "final states that are in ompl goals\n";
            // goal->print();
        }

        else if (task->goal.type == TaskDef::Goal::DUMMY_PAIR || task->goal.type == TaskDef::Goal::CLLBACK)
        {
            goal = ob::GoalPtr(new Goal(task->spaceInformationPtr, task, (double)task->goal.tolerance));
        }
        task->problemDefinitionPtr->setGoal(goal);

        ob::PlannerPtr planner_new = ob::PlannerPtr(new og::RRT(task->spaceInformationPtr));
        task->planner = planner_new;

        // task->planner = plannerFactory(task->algorithm, task->spaceInformationPtr);

        if (!task->planner)
        {
            // throw std::runtime_error("Invalid motion planning algorithm.");
            RCLCPP_INFO(_node->get_logger(), "Invalid motion planning algorithm.");
            return if_error_appear_return_zero;
        }
        task->planner->setProblemDefinition(task->problemDefinitionPtr);

        // find shortest path
        float path_size = INT16_MAX;
        ArmConfiguration shortest_path;

        for (int i = 0; i < _OMPL_compute_trials; i++)
        {
            ArmConfiguration calculated_path;

            int time_to_compute = _max_time * (1 + i); // in seconds

            ob::PlannerStatus solved = task->planner->solve(time_to_compute);

            if (!solved)
            {
                std::cout << "PROBLEM CANNOT BE SOLVED\n";
                continue;
            }

            if (solved != ob::PlannerStatus::EXACT_SOLUTION)
            {
                std::cout << "CANNOT FIND EXACT SOLUTION\n";
                continue;
            }

            // if solved stop calculating
            if (solved)
                i = _OMPL_compute_trials;

            const ob::PathPtr &path_ = task->problemDefinitionPtr->getSolutionPath();
            og::PathGeometric &path = static_cast<og::PathGeometric &>(*path_);
            og::PathSimplifierPtr pathSimplifier(new og::PathSimplifier(task->spaceInformationPtr));

            if (_max_simplification_time < -std::numeric_limits<double>::epsilon())
                pathSimplifier->simplifyMax(path);
            else
                pathSimplifier->simplify(path, _max_simplification_time);

            // int path_wp = path.getStateCount();
            // std::cout << "OMPL path before interpolate : " << path_wp << "\n";

            // ob::State *start_state = path.getState(0);
            // ob::State *goal_state = path.getState(path_wp - 1);

            // std::cout << "start state\n";
            // task->spaceInformationPtr->printState(start_state);
            // std::cout << "goal state\n";
            // task->spaceInformationPtr->printState(goal_state);

            // ob::RealVectorStateSpace::StateType *place_state = goal_state->as<ob::RealVectorStateSpace::StateType>();
            // task->spaceInformationPtr->printState(place_state);

            path.interpolate(_min_path_points);

            // path_wp = path.getStateCount();
            // std::cout << "OMPL path after interpolate : " << path.getStateCount() << "\n";

            // ob::State *start_state2 = path.getState(0);
            // ob::State *goal_state2 = path.getState(path_wp - 1);

            // std::cout << "start state\n";
            // task->spaceInformationPtr->printState(start_state2);
            // std::cout << "gaol state\n";
            // task->spaceInformationPtr->printState(goal_state2);

            for (size_t i = 0; i < path.getStateCount(); i++)
            {
                const ob::StateSpace::StateType *s = path.getState(i);
                std::vector<double> v;
                task->stateSpacePtr->copyToReals(v, s);

                for (size_t j = 0; j < v.size(); j++)
                    calculated_path.push_back((float)v[j]);
            }

            float current_size = _getPathLength(calculated_path);
            if (path_size > current_size)
            {
                shortest_path = calculated_path;
                path_size = current_size;
            }
        }

        // ???? should be?
        //_tasks[task->header.handle] = task;
        //_tasks.erase(task->header.handle);

        delete task;
        return shortest_path;
    }
}

int Plugin::_createStateSpace(int joint_number) // TODO bounds change
{
    if (_metric[joint_number] <= 0)
        throw std::runtime_error("State component weight must be positive.");

    StateSpaceDef *statespace = new StateSpaceDef();
    statespace->header.destroyAfterSimulationStop = simGetSimulationState() != sim_simulation_stopped;
    statespace->header.handle = _nextStateSpaceHandle++;
    statespace->header.name = _joint_names[joint_number];
    statespace->type = sim_ompl_statespacetype_joint_position;
    statespace->objectHandle = _joint_handles[joint_number];

    // for (size_t i = 0; i < _bounds[joint_number].size(); i++)
    statespace->boundsLow.push_back(_bounds[joint_number].boundsLow);

    // for (size_t i = 0; i < _bounds[joint_number].boundsHigh.size(); i++)
    statespace->boundsHigh.push_back(_bounds[joint_number].boundsHigh);

    if (joint_number < 3)
        statespace->defaultProjection = joint_number + 1;
    else
        statespace->defaultProjection = 0;

    statespace->weight = 1.0;
    statespace->refFrameHandle = -1;
    statespace->dubinsTurningRadius = 0.05;
    statespace->dubinsIsSymmetric = false;
    _statespaces[statespace->header.handle] = statespace;

    return statespace->header.handle;
}

std::vector<float> Plugin::_getArmState() //!ok
{
    std::vector<float> arm_state;
    for (auto joint_handle : _joint_handles)
    {
        float position = 0;
        simGetJointPosition(joint_handle, &position);
        arm_state.push_back(position);
    }
    return arm_state;
}

SIM_PLUGIN(PLUGIN_NAME, 1, Plugin)
#include "stubsPlusPlus.cpp"
