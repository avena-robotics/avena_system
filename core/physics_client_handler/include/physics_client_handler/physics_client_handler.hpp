#ifndef PHYSICS_CLIENT_HANDLER__PHYSICS_CLIENT_HANDLER_HPP_
#define PHYSICS_CLIENT_HANDLER__PHYSICS_CLIENT_HANDLER_HPP_

// ___CPPP___
#include <tuple>

// ___Avena___
#include <helpers_commons/helpers_commons.hpp>

// ___Package___
#include "physics_client_handler/commons.hpp"
#include "physics_client_handler/visibility_control.h"

namespace physics_client_handler
{
  class PhysicsClientHandler
  {
  public:
    PhysicsClientHandler() = default;
    PhysicsClientHandler(const helpers::commons::RobotInfo &robot_info, const rclcpp::Logger &logger);
    ~PhysicsClientHandler();
    bool inCollision(const Obstacles &obstacles);
    bool inCollision();
    const Obstacles &getCollisionObjectsHandles();
    void setJointStates(const ArmConfiguration &joint_states);
    ArmConfiguration getJointStates();
    void drawCoordinateAxes(const Eigen::Affine3d &pose);
    void cleanDebugItems();
    bool isSceneValid();
    std::optional<Eigen::Affine3d> getEndEffectorPose();
    void syncWithPhysicsServer();
    bullet_client::b3RobotSimulatorClientAPI::SharedPtr getPhysicsClient() const { return _bullet_client; }
    ArmConfiguration computeIk(const Eigen::Affine3d &end_effector_pose,
                               const std::vector<Limits> &joint_limits,
                               const Obstacles &collision_objects);

    using SharedPtr = std::shared_ptr<PhysicsClientHandler>;

  private:
    // ___Methods___
    void _readSceneInfoFromPhysicsServer(const helpers::commons::RobotInfo &robot_info);

    // ___Attributes___
    bullet_client::b3RobotSimulatorClientAPI::SharedPtr _bullet_client;
    int _robot_idx = INVALID_HANDLE;
    int _end_effector_idx = INVALID_HANDLE;
    std::vector<int> _joint_handles;
    std::vector<int> _q_indices;
    size_t _num_degrees_of_freedom;
    Obstacles _obstacles;
    helpers::commons::RobotInfo _robot_info;
    rclcpp::Logger _logger;
    
    // ___Constants___
    static constexpr float SAFETY_DISTANCE = 0.005; // in meters
    static constexpr int CONTACT_NUMBER_ALLOWED = 1;
    static constexpr double POSITION_THRESHOLD = 1e-4;
    static constexpr double ORIENTATION_THRESHOLD = 1e-3;
  };

} // namespace physics_client_handler

#endif // PHYSICS_CLIENT_HANDLER__PHYSICS_CLIENT_HANDLER_HPP_
