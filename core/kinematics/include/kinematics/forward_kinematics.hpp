#ifndef KINEMATICS__FORWARD_KINEMATICS_HPP_
#define KINEMATICS__FORWARD_KINEMATICS_HPP_

// ___CPP___
#include <Eigen/Dense>

// ___Avena___
#include <bullet_client/b3RobotSimulatorClientAPI.h>
#include <helpers_commons/helpers_commons.hpp>
#include <helpers_vision/helpers_vision.hpp>

// ___Package___
#include "ik_fast/ik_franka.hpp"
#include "ik_fast/ik_avena.hpp"
#include "kinematics/commons.hpp"
#include "kinematics/visibility_control.h"
#include "physics_client_handler/physics_client_handler.hpp"

namespace kinematics
{
  class ForwardKinematics
  {
  public:
    explicit ForwardKinematics(physics_client_handler::PhysicsClientHandler::SharedPtr physics_server_handler,
                               helpers::commons::RobotInfo &robot_info,
                               const rclcpp::Logger &logger);
    virtual ~ForwardKinematics() = default;
  
    /**
     * @brief 
     * 
     * @param joint_states 
     * @param in_robot_base_frame 
     * @return Eigen::Affine3d 
     */
    Eigen::Affine3d computeFk(const ArmConfiguration &joint_states,
                               bool in_robot_base_frame = false);

    /**
     * Change reference frame of end effector (default: "world")
     * @param reference_frame frame in which end effector pose is
     */
    void setReferenceFrame(const Eigen::Affine3d &reference_frame);

    using SharedPtr = std::shared_ptr<ForwardKinematics>;

  private:
    // ___Attributes___
    Eigen::Affine3d _reference_frame;
    physics_client_handler::PhysicsClientHandler::SharedPtr _physics_server_handler;
    helpers::commons::RobotInfo _robot_info;
    rclcpp::Logger _logger;
  };

} // namespace kinematics

#endif // KINEMATICS__FORWARD_KINEMATICS_HPP_
