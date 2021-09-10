#ifndef KINEMATICS__KINEMATICS_HPP_
#define KINEMATICS__KINEMATICS_HPP_

// ___Package___
#include "kinematics/inverse_kinematics.hpp"
#include "kinematics/forward_kinematics.hpp"

namespace kinematics
{
  class Kinematics
  {
  public:
    explicit Kinematics(physics_client_handler::PhysicsClientHandler::SharedPtr physics_server_handler,
                        helpers::commons::RobotInfo &robot_info,
                        const rclcpp::Logger &logger);
    ~Kinematics() = default;

    InverseKinematics::SharedPtr ik;
    ForwardKinematics::SharedPtr fk;

    using SharedPtr = std::shared_ptr<Kinematics>;
    using UniquePtr = std::unique_ptr<Kinematics>;
  };

} // namespace kinematics

#endif // KINEMATICS__KINEMATICS_HPP_
