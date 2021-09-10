#include "kinematics/kinematics.hpp"

namespace kinematics
{
    Kinematics::Kinematics(physics_client_handler::PhysicsClientHandler::SharedPtr physics_server_handler,
                           helpers::commons::RobotInfo &robot_info,
                           const rclcpp::Logger &logger)
    {
        ik = std::make_shared<InverseKinematics>(physics_server_handler, robot_info, logger);
        fk = std::make_shared<ForwardKinematics>(physics_server_handler, robot_info, logger);
    }
} // namespace kinematics
