#ifndef GENERATE_PATH__GENERATE_PATH_HPP_
#define GENERATE_PATH__GENERATE_PATH_HPP_

// ___Package___
#include "generate_path/visibility_control.h"
#include "generate_path/commons.hpp"
#include "generate_path/planners/factory_planner.hpp"

namespace generate_path
{
  class GeneratePath
  {
  public:
    explicit GeneratePath(rclcpp::Node::SharedPtr node, physics_client_handler::PhysicsClientHandler::SharedPtr physics_client_handler) noexcept(false);
    virtual ~GeneratePath() = default;
    GeneratedPath::SharedPtr generatePath(const MovementSequence &movement_sequence) noexcept(false);

    using UniquePtr = std::unique_ptr<GeneratePath>;
    using SharedPtr = std::shared_ptr<GeneratePath>;

  private:
    // ___Methods___
    ArmConfiguration _getJointStatesFromTopic(const sensor_msgs::msg::JointState::SharedPtr &joint_states);
    ReturnCode _getParametersFromServer();
    ReturnCode _checkJointLimits(const ArmConfiguration &joint_states, const std::vector<Limits> &joint_limits);
    void _convertPathSegmentToTrajectoryMsg(const std::vector<ArmConfiguration> &path, trajectory_msgs::msg::JointTrajectory &path_segment);
    ReturnCode _readSceneInfoFromPhysicsServer();
    void _validateArmConfiguration(const ArmConfiguration &joint_state) noexcept(false);
    sensor_msgs::msg::JointState::SharedPtr _getCurrentJointStates();

    // ___Attributes___
    rclcpp::Node::SharedPtr _node;
    physics_client_handler::PhysicsClientHandler::SharedPtr _physics_client_handler;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_state_sub;
    std::mutex _current_joint_states_mtx;
    sensor_msgs::msg::JointState::SharedPtr _current_joint_states;
    helpers::commons::RobotInfo _robot_info;
    kinematics::Kinematics::SharedPtr _kinematics_engine;
  };

} // namespace generate_path

#endif // GENERATE_PATH__GENERATE_PATH_HPP_
