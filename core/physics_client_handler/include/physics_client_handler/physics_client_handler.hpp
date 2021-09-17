#ifndef PHYSICS_CLIENT_HANDLER__PHYSICS_CLIENT_HANDLER_HPP_
#define PHYSICS_CLIENT_HANDLER__PHYSICS_CLIENT_HANDLER_HPP_

// ___Package___
#include "physics_client_handler/commons.hpp"

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
    void cleanDebugItems();
    bool isSceneValid();
    std::optional<Eigen::Affine3d> getEndEffectorPose();
    void syncWithPhysicsServer();
    bullet_client::b3RobotSimulatorClientAPI::SharedPtr getPhysicsClient() const { return _bullet_client; }
    ArmConfiguration computeIk(const Eigen::Affine3d &end_effector_pose,
                               const std::vector<Limits> &joint_limits,
                               const Obstacles &collision_objects,
                               const std::chrono::duration<double> &timeout = std::chrono::milliseconds(500));
    std::vector<ArmConfiguration> computeAllIk(const Eigen::Affine3d &end_effector_pose,
                                               const std::vector<Limits> &joint_limits,
                                               const Obstacles &collision_objects,
                                               const std::chrono::duration<double> &timeout = std::chrono::milliseconds(500));
    void refreshConnection();
    int createOctomap(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, float grid_size);
    void removeCollisions();

    /**
     * @brief Get the Jacobian object
     * The returned jacobian has dimensions equal to:
     * number of rows: 6
     * number of columns: degrees of freedom
     * 
     * Format of returned jacobian:
     *            | J_v |
     * Jacobian = |     |
     *            | J_w |
     * Where: 
     * J_v is linear velocity Jacobian (dims: 3 x DOF)
     * J_w is angular velocity Jacobian (dims: 3 x DOF)
     * @param joint_states Joint values for which Jacobian should be calculated
     * @return std::optional<Eigen::MatrixXd> 
     */
    std::optional<Eigen::MatrixXd> getJacobian(const ArmConfiguration &joint_states);

    using SharedPtr = std::shared_ptr<PhysicsClientHandler>;

    // void drawCoordinateAxes(const Eigen::Affine3d &pose, double scale=0.2);

    template <typename T>
    void drawCoordinateAxes(const Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> &pose, double scale = 0.2)
    {
      Eigen::Matrix<T, 3, 1> position(pose.translation());
      Eigen::Quaternion<T> orientation(pose.rotation());
      Eigen::Matrix<T, 3, 1> x_shift = orientation.toRotationMatrix().col(0) * scale;
      Eigen::Matrix<T, 3, 1> pos_x = position + x_shift;

      Eigen::Matrix<T, 3, 1> y_shift = orientation.toRotationMatrix().col(1) * scale;
      Eigen::Matrix<T, 3, 1> pos_y = position + y_shift;

      Eigen::Matrix<T, 3, 1> z_shift = orientation.toRotationMatrix().col(2) * scale;
      Eigen::Matrix<T, 3, 1> pos_z = position + z_shift;

      b3RobotSimulatorAddUserDebugLineArgs line_args;
      line_args.m_colorRGB[0] = 1;
      line_args.m_colorRGB[1] = 0;
      line_args.m_colorRGB[2] = 0;
      btVector3 from(position.x(), position.y(), position.z());
      btVector3 to_x(pos_x.x(), pos_x.y(), pos_x.z());
      _bullet_client->addUserDebugLine(from, to_x, line_args);
      line_args.m_colorRGB[0] = 0;
      line_args.m_colorRGB[1] = 1;
      line_args.m_colorRGB[2] = 0;
      btVector3 to_y(pos_y.x(), pos_y.y(), pos_y.z());
      _bullet_client->addUserDebugLine(from, to_y, line_args);
      line_args.m_colorRGB[0] = 0;
      line_args.m_colorRGB[1] = 0;
      line_args.m_colorRGB[2] = 1;
      btVector3 to_z(pos_z.x(), pos_z.y(), pos_z.z());
      _bullet_client->addUserDebugLine(from, to_z, line_args);
    }

  private:
    // ___Methods___
    void _readSceneInfoFromPhysicsServer(const helpers::commons::RobotInfo &robot_info);
    int _createMeshFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, const float &grid_size, std::vector<pcl::Vertices> &out_triangles);

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
