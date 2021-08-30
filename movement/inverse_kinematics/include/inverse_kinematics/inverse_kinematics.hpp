#ifndef INVERSE_KINEMATICS__INVERSE_KINEMATICS_HPP_
#define INVERSE_KINEMATICS__INVERSE_KINEMATICS_HPP_

// ___CPP___
#include <Eigen/Dense>

// ___Avena___
#include <bullet_client/b3RobotSimulatorClientAPI.h>
#include <helpers_commons/helpers_commons.hpp>
#include <helpers_vision/helpers_vision.hpp>

// ___Package___
#include "inverse_kinematics/ik_avena.hpp"
#include "inverse_kinematics/ik_franka.hpp"
#include "inverse_kinematics/commons.hpp"
#include "inverse_kinematics/visibility_control.h"
#include "physics_client_handler/physics_client_handler.hpp"

namespace inverse_kinematics
{
  class InverseKinematics
  {
  public:
    explicit InverseKinematics(physics_client_handler::PhysicsClientHandler::SharedPtr physics_server_handler,
                               helpers::commons::RobotInfo &robot_info,
                               const rclcpp::Logger &logger = rclcpp::get_logger("inverse_kinematics"));
    virtual ~InverseKinematics() = default;

    /**
     * Compute arm configuration given end effector pose (using joints 
     * limits provided in URDF with ranged tightened with joint_ranges_coeff
     * passed in constructor)
     * @param end_effector_pose cartesian pose of end effector in world frame
     * @param in_robot_base_frame whether end effector pose is in robot base frame of reference
     */
    ArmConfiguration computeIk(const Eigen::Affine3d &end_effector_pose, bool in_robot_base_frame = false);

    /**
     * Compute arm configuration given end effector pose 
     * and user provided joint limits to explore sub-space joint ranges.
     * @param end_effector_pose cartesian pose of end effector in world frame
     * @param joint_limits lower and upper values of joints allowed (in radians) 
     * @param in_robot_base_frame whether end effector pose is in robot base frame of reference
     */
    ArmConfiguration computeIk(const Eigen::Affine3d &end_effector_pose,
                               const std::vector<Limits> &joint_limits,
                               bool in_robot_base_frame = false);

    /**
     * Change reference frame of end effector (default: "world")
     * @param reference_frame frame in which end effector pose is
     */
    void setReferenceFrame(const Eigen::Affine3d &reference_frame);

    using SharedPtr = std::shared_ptr<InverseKinematics>;

  private:
    // /**
    //  * Change range of joint limits by applying software limits to avoid hitting 
    //  * hardware limits because of physical robot inertias
    //  * @param joint_range_coeff values from 0.0 - 1.0 how much scale down range for joint limits
    //  */
    // void _updateJointLimits(const float &joint_range_coeff);

    ReturnCode _validateArmConfiguration(const ArmConfiguration &joint_state,
                                         const std::vector<Limits> &joint_limits,
                                         const std::vector<int> &obstacles);
    ReturnCode _checkJointLimits(const ArmConfiguration &joint_states, const std::vector<Limits> &joint_limits);

    // ___Attributes___
    Eigen::Affine3d _reference_frame;
    helpers::commons::RobotInfo _robot_info;
    physics_client_handler::PhysicsClientHandler::SharedPtr _physics_server_handler;
    rclcpp::Logger _logger;
  };

} // namespace inverse_kinematics

#endif // INVERSE_KINEMATICS__INVERSE_KINEMATICS_HPP_
