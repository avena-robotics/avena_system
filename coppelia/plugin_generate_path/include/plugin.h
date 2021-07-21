
#ifndef SIM_PATH_GENERATOR_H
#define SIM_PATH_GENERATOR_H

#include "ompl_test.h"

////////////////////////

#include <robot_state_publisher/robot_state_publisher.hpp>

// ___COPPELIA PLUGINS___
#include "config.h"
#include "simPlusPlus/Plugin.h"
#include "stubs.h"

// ___CPP___
#include <cmath>
// #include <math.h>
#include <memory>
#include <string>
#include <sstream>
#include <atomic>

#include <eigen3/Eigen/Eigen>

//___ROS___
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <custom_interfaces/msg/place.hpp>
#include <custom_interfaces/msg/grasp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <custom_interfaces/action/simple_action.hpp>
#include <custom_interfaces/action/generate_path_pick_action.hpp>
#include <custom_interfaces/action/generate_path_place_action.hpp>
#include <custom_interfaces/action/generate_path_pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
// #include <custom_interfaces/msg/generated_path_segments.hpp>

#include "helpers_commons/helpers_commons.hpp"

#define PLUGIN_NAME "PathGenerator"
#define PLUGIN_VERSION 1

using namespace std::chrono_literals;
namespace ob = ompl::base;
namespace og = ompl::geometric;

struct TargetMatrix
{
    std::vector<std::vector<float>> post_start;
    std::vector<std::vector<float>> pre_target;
    std::vector<std::vector<float>> final_target;
    int selected_item_id;
};

struct Bounds
{
    float boundsLow;
    float boundsHigh;
};

enum Job
{
    PICK,
    PLACE,
    HOME,
    POSE,
};

using GeneratePath = custom_interfaces::action::SimpleAction;
using GoalHandleGeneratePath = rclcpp_action::ServerGoalHandle<GeneratePath>;

using GeneratePathPose = custom_interfaces::action::GeneratePathPose;
using GoalHandleGeneratePathPose = rclcpp_action::ServerGoalHandle<GeneratePathPose>;

using GeneratePathPick = custom_interfaces::action::GeneratePathPickAction;
using GoalHandleGeneratePathPick = rclcpp_action::ServerGoalHandle<GeneratePathPick>;

using GeneratePathPlace = custom_interfaces::action::GeneratePathPlaceAction;
using GoalHandleGeneratePathPlace = rclcpp_action::ServerGoalHandle<GeneratePathPlace>;

using ArmConfiguration = std::vector<float>;

class Plugin : public sim::Plugin
{

public:
    Plugin();
    virtual ~Plugin() = default;
    void onStart();
    void onInstancePass(const sim::InstancePassFlags &flags, bool first) override;
    // void getInput(getInput_in *in, getInput_out *out);
    // void selectJob(selectJob_in *in, selectJob_out *out);

    // void readTargetPlaceMatrix(readTargetPlaceMatrix_in *in, readTargetPlaceMatrix_out *out);
    // void isGenerateFinished(isGenerateFinished_in *in, isGenerateFinished_out *out);

private:
    void _setTarget(const float pose[3], const float quaternion[4], const std::string name);

    TargetMatrix _setCartesianPlace();
    TargetMatrix _setCartesianPick();
    TargetMatrix _setCartesianPose();

    // helpers::Watchdog::SharedPtr _watchdog;
    int _getParametersFromServer();

    // luaaaa
    trajectory_msgs::msg::JointTrajectory _selectPath(std::vector<std::vector<float>> goal_states);
    trajectory_msgs::msg::JointTrajectory _createPathData(const std::vector<float> path);

    void _initialize();
    float _getDistanceBetweenStates(std::vector<float> arm_state1, std::vector<float> arm_state2);
    float _getPathLength(std::vector<float> path);
    std::vector<float> _findPath(std::vector<std::vector<float>> goal_states, Job job);
    int _createStateSpace(int joint_number);
    std::vector<float> _getArmState();
    std::vector<std::vector<float>> _findCollisionFreeArmStates(std::vector<std::vector<float>> matrix_poses);
    std::vector<std::vector<float>> _findCollisionFreeArmStates(TargetMatrix matrix_poses, bool no_collision);

    void _setArmState(std::vector<float> state);
    void _setCurrentArmState();
    std::vector<float> _generateIkPath(std::vector<std::vector<float>> matrix, const size_t &nr_path_points);
    std::vector<float> _freeCollisionIkPath(std::vector<std::vector<float>> matrix);
    void _setLastPathConfig(std::vector<float> path);
    void _openGripper();
    void _savePickState();

    // end lua functions
    bool _generatePath(Job job);
    rclcpp::Node::SharedPtr _node;
    // ___Pick___
    rclcpp_action::GoalResponse _handleGoalPick(const rclcpp_action::GoalUUID &uuid,
                                                std::shared_ptr<const GeneratePathPick::Goal> goal);
    rclcpp_action::CancelResponse _handleCancelPick(const std::shared_ptr<GoalHandleGeneratePathPick> goal_handle);
    void _handleAcceptedPick(const std::shared_ptr<GoalHandleGeneratePathPick> goal_handle);
    void _executePick(const std::shared_ptr<GoalHandleGeneratePathPick> goal_handle);

    // ___Place___
    rclcpp_action::GoalResponse _handleGoalPlace(const rclcpp_action::GoalUUID &uuid,
                                                 std::shared_ptr<const GeneratePathPlace::Goal> goal);
    rclcpp_action::CancelResponse _handleCancelPlace(const std::shared_ptr<GoalHandleGeneratePathPlace> goal_handle);
    void _handleAcceptedPlace(const std::shared_ptr<GoalHandleGeneratePathPlace> goal_handle);
    void _executePlace(const std::shared_ptr<GoalHandleGeneratePathPlace> goal_handle);

    // ___Home___
    rclcpp_action::GoalResponse _handleGoalHome(const rclcpp_action::GoalUUID &uuid,
                                                std::shared_ptr<const GeneratePath::Goal> goal);
    rclcpp_action::CancelResponse _handleCancelHome(const std::shared_ptr<GoalHandleGeneratePath> goal_handle);
    void _handleAcceptedHome(const std::shared_ptr<GoalHandleGeneratePath> goal_handle);
    void _executeHome(const std::shared_ptr<GoalHandleGeneratePath> goal_handle);

    // // ___Go to point___
    rclcpp_action::GoalResponse _handleGoalPose(const rclcpp_action::GoalUUID &uuid,
                                                std::shared_ptr<const GeneratePathPose::Goal> goal);
    rclcpp_action::CancelResponse _handleCancelPose(const std::shared_ptr<GoalHandleGeneratePathPose> goal_handle);
    void _handleAcceptedPose(const std::shared_ptr<GoalHandleGeneratePathPose> goal_handle);
    void _executePose(const std::shared_ptr<GoalHandleGeneratePathPose> goal_handle);

    rclcpp_action::Server<GeneratePathPick>::SharedPtr _action_server_pick;
    rclcpp_action::Server<GeneratePathPlace>::SharedPtr _action_server_place;
    rclcpp_action::Server<GeneratePath>::SharedPtr _action_server_home;
    rclcpp_action::Server<GeneratePathPose>::SharedPtr _action_server_pose;

    // void _getPickTargetCallback(const custom_interfaces::msg::Grasp::SharedPtr target_pick_msg);
    void _getPlaceTargetCallback(const custom_interfaces::msg::Place::SharedPtr target_place_msg);

    custom_interfaces::msg::Place _target_place_data;
    custom_interfaces::msg::Grasp _target_pick_data;
    geometry_msgs::msg::Pose _target_pose_data;

    trajectory_msgs::msg::JointTrajectory _path_msg;
    trajectory_msgs::msg::JointTrajectory _path_to_pick;
    trajectory_msgs::msg::JointTrajectory _pick_to_place;
    trajectory_msgs::msg::JointTrajectory _path_to_pose;
    std::vector<std::string> _frames;

    // void _broadcastPath();
    rclcpp::TimerBase::SharedPtr _timer_pub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _pub_path;

    sensor_msgs::msg::JointState::SharedPtr _joint_states_data;

    std::vector<float> _pick_state;
    void _detachItem();

    std::vector<float> _getShiftAlongGripperAxisMatrix(std::vector<float> matrix);
    std::vector<float> _getShiftAlongZAxisMatrix(std::vector<float> matrix);
    // bool _check_4_5_links_collision_distance(float treshold);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_state_sub;

    bool _parameters_read;
    helpers::commons::RobotInfo _robot_info;
    helpers::commons::RobotInfo _collision_robot_info;

    //?
    int _is_generate_finished;
    int _handle;
    int _frame_handle;
    int _job{0};
    std::vector<float> _matrix_pick;
    std::vector<float> _matrix_place;

    float constrain_value;
    int ik_trials_number;
    int max_final_states;
    int ompl_compare_trials;
    int min_path_points;
    int max_time;
    int max_simplification_time;

    int _max_final_configs{6};     // number of maximum proper and different joint state configs that approach final
                                   // point
    int _IK_trials_number{60};     // number of trials of calculating final joint state config
    int _OMPL_compute_trials{6};   // number of trials of OMPL computing for each goal config
    int _set_constraint{0};        // set a constraint in z axis 1 - on, 0 -false
    int _constraint_tolerance{20}; // tolerance of the constraint: less - more accurate (range 10-50) (less then
                                   // 10 extends time very much)

    // variables from lua

    int _dummyWorld_handle;
    std::vector<int> _joint_handles;
    int _left_finger_handle;  // TODO: Robot specific
    int _right_finger_handle; // TODO: Robot specific

    std::vector<float> _home_pose;
    // std::vector<float> _home_pose{2, 0.8, 0.8, -0.5, -1.5, -1.5};
    std::vector<float> _metric;
    std::vector<std::string> _joint_names;
    int _ikGroup;
    int _ikTarget;
    int _ikTip;
    int _robotCollectionHandle;
    int _sceneCollectionHandle;
    int _collisionRobotCollectionHandle;
    int _hand;
    int _desk;
    int _parent_handle;
    int _world_frame;
    int _home_target;
    int _pose_target_handle;
    int _joint_numbers;
    int _min_path_points = 150;
    int _max_time = 2;
    int _max_simplification_time = -1;
    int _nextTaskHandle = 1000;
    int _nextStateSpaceHandle = 9000;
    float _x_shift = -0.05;
    float _z_shift = 0.05;

    std::map<int, TaskDef *> _tasks;
    std::map<int, StateSpaceDef *> _statespaces;
    std::vector<Bounds> _bounds;
    std::vector<double> _world_rotation;
    std::vector<int> _collisionPairHandles;

    const float JOINT_STATE_DIFF = 1e-3;

    const std::map<std::string, std::vector<float>> METRICS = {
        {"avena", {0.85, 0.5, 0.4, 0.2, 0.1, 0.2}},
        {"franka", {0.85, 0.5, 0.4, 0.2, 0.1, 0.2, 0.1}},
    };

    const std::map<std::string, std::vector<float>> HOME_JOINTS_STATE = {
        {"avena", {0, 0, 0, 0, 0, 0}},
        {"franka", {0, 0, 0, -0.78539818525314, 0, 1.57, -1.57}},
    };
};

#endif