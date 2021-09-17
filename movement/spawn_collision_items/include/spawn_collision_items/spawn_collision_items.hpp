#ifndef SPAWN_COLLISION_ITEMS__SPAWN_COLLISION_ITEMS_HPP_
#define SPAWN_COLLISION_ITEMS__SPAWN_COLLISION_ITEMS_HPP_

// ___Package___
#include "spawn_collision_items/commons.hpp"

namespace spawn_collision_items
{
  class SpawnCollisionItems
  {
  public:
    explicit SpawnCollisionItems(rclcpp::Node::SharedPtr node, physics_client_handler::PhysicsClientHandler::SharedPtr physics_client_handler) noexcept(false);
    virtual ~SpawnCollisionItems() = default;
    void spawnOctomap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene_octomap);

    using UniquePtr = std::unique_ptr<SpawnCollisionItems>;
    using SharedPtr = std::shared_ptr<SpawnCollisionItems>;

  private:
    // ___Methods___
    ReturnCode _getParametersFromServer();
    void _clearCollisions();

    // ___Attributes___
    rclcpp::Node::SharedPtr _node;
    physics_client_handler::PhysicsClientHandler::SharedPtr _physics_client_handler;
  };

} // namespace spawn_collision_items

#endif // SPAWN_COLLISION_ITEMS__SPAWN_COLLISION_ITEMS_HPP_
