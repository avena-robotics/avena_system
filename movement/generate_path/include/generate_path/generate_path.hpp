#ifndef GENERATE_PATH__GENERATE_PATH_HPP_
#define GENERATE_PATH__GENERATE_PATH_HPP_

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <opencv2/opencv.hpp>

// ___KDL___
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <urdf/model.h>

// ___OMPL___
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/geometric/GeneticSearch.h>



// ___Avena___
#include <custom_interfaces/action/generate_path_pose.hpp>
#include <custom_interfaces/msg/generated_path.hpp>
#include <helpers_commons/helpers_commons.hpp>
#include <helpers_vision/helpers_vision.hpp>
#include <bullet_client/b3RobotSimulatorClientAPI.h>

// ___Package___
#include "generate_path/visibility_control.h"
#include "generate_path/commons.hpp"
#include "generate_path/planner.hpp"

#include "generate_path/ik_franka.hpp"

namespace generate_path
{
  using GeneratePathPose = custom_interfaces::action::GeneratePathPose;
  using GoalHandleGeneratePathPose = rclcpp_action::ServerGoalHandle<GeneratePathPose>;

  class GeneratePath : public rclcpp::Node, public helpers::WatchdogInterface
  {
  public:
    explicit GeneratePath(const rclcpp::NodeOptions &options);
    virtual ~GeneratePath();
    virtual void initNode() override;
    virtual void shutDownNode() override;

  private:
    // ___Methods___
    // ___Go to end effector pose___
    rclcpp_action::GoalResponse _handleGoalPose(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GeneratePathPose::Goal> goal);
    rclcpp_action::CancelResponse _handleCancelPose(const std::shared_ptr<GoalHandleGeneratePathPose> goal_handle);
    void _handleAcceptedPose(const std::shared_ptr<GoalHandleGeneratePathPose> goal_handle);
    void _executePose(const std::shared_ptr<GoalHandleGeneratePathPose> goal_handle);
    ReturnCode _initialize();
    ReturnCode _shutdown();
    ArmConfiguration _getJointStatesFromTopic(const sensor_msgs::msg::JointState::SharedPtr &joint_states);
    ReturnCode _getParametersFromServer();
    ReturnCode _checkJointLimits(const ArmConfiguration &joint_states, const std::vector<Limits> &joint_limits);
    static double _calculateDistanceEndEffectorPosToGoalPos(const Eigen::Affine3d &end_effector_pose, const Eigen::Affine3d &goal_end_effector_pose);
    static double _calculateDistanceEndEffectorOrienToGoalOrien(const Eigen::Affine3d &end_effector_pose, const Eigen::Affine3d &goal_end_effector_pose);
    Eigen::Affine3d _getEndEffectorPose();
    void _convertPathSegmentToTrajectoryMsg(const std::vector<ArmConfiguration> &path, trajectory_msgs::msg::JointTrajectory &path_segment);
    std::tuple<ArmConfiguration, std::string> _calculateIK(const PathPlanningInput &path_planning_input);
    ReturnCode _readSceneInfoFromPhysicsServer();
    void _updateJointLimits();
    void _setJointStates(const ArmConfiguration &joint_states);
    void _drawCoordinateAxes(const Eigen::Affine3d &pose);
    ReturnCode _validateConfiguration(const PathPlanningInput &path_planning_input, const ArmConfiguration &joint_state, std::string &error_message);
    // ReturnCode _

    // ArmConfiguration _calculateIKWithKDL(const ArmConfiguration &initial_state, const Eigen::Affine3d &end_effector_pose);

    // ___Attributes___
    helpers::Watchdog::SharedPtr _watchdog;
    rclcpp_action::Server<GeneratePathPose>::SharedPtr _action_server_pose;
    rclcpp::Publisher<custom_interfaces::msg::GeneratedPath>::SharedPtr _generated_path_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_state_sub;
    std::mutex _current_joint_states_mtx;
    sensor_msgs::msg::JointState::SharedPtr _current_joint_states;
    helpers::commons::RobotInfo _robot_info;
    SceneInfo::SharedPtr _scene_info;
    geometry_msgs::msg::TransformStamped _robot_base_tf;
     
    // /**
    //  * @brief Inverse kinematic limits are tighter than path planning ones just to be sure than planning does not go out of bounds
    //  */
    // std::vector<Limits> _ik_joint_limits;

    /**
     * @brief ID in physics server of table and all static things which are not changing e.g. camera stands, artificial walls for collisions
     * The reason to keep it separately and not as an generic obstacle is to easily keep track of changing obstacles when new moving items
     * appear on the table.
     */
    int _table_idx;

    const float _safety_range = 0.003;
    const int _contact_number_allowed = 1;
    const double _end_effector_position_offset = 0.005; // In meters; distance from calculated end effector position to goal end effector
    const double _end_effector_orientation_offset = 0.01; // In radians; sum of all axes
  };

} // namespace generate_path

#endif // GENERATE_PATH__GENERATE_PATH_HPP_
