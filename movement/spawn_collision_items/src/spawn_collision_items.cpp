#include "spawn_collision_items/spawn_collision_items.hpp"

namespace spawn_collision_items
{
    SpawnCollisionItems::SpawnCollisionItems(rclcpp::Node::SharedPtr node, physics_client_handler::PhysicsClientHandler::SharedPtr physics_client_handler)
        : _node(node),
          _physics_client_handler(physics_client_handler)
    {
        if (!_physics_client_handler)
            throw SpawnCollisionItemsError("Physics client handler is not initialized");

        if (_getParametersFromServer() != ReturnCode::SUCCESS)
            throw SpawnCollisionItemsError("Cannot read parameters from server");
    }

    void SpawnCollisionItems::spawnOctomap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene_octomap)
    {
        helpers::Timer timer(__func__, _node->get_logger());
        RCLCPP_INFO(_node->get_logger(), "Removing current collisions");
        {
            helpers::Timer timer("Removing current collisions", _node->get_logger());
            _clearCollisions();
        }
        RCLCPP_INFO(_node->get_logger(), "Spawning scene octomap");

        // TODO: Change this to value read from parameter server if necessary
        _physics_client_handler->createOctomap(scene_octomap, 0.025);
        // RCLCPP_INFO_STREAM(_node->get_logger(), "Octomap unique ID: " << octomap_handler);
    }

    void SpawnCollisionItems::_clearCollisions()
    {
        _physics_client_handler->removeCollisions();
    }

    ReturnCode SpawnCollisionItems::_getParametersFromServer()
    {
        RCLCPP_INFO(_node->get_logger(), "[Spawn collision items] Reading parameters from the server");

        nlohmann::json labels = helpers::commons::getParameter("labels");
        if (labels.empty())
            return ReturnCode::FAILURE;

        // TODO: Parse labels... or not??? Check this
        // RCLCPP_INFO_STREAM(_node->get_logger(), "Labels:\n" << std::setw(2) << labels);

        RCLCPP_INFO(_node->get_logger(), "[Spawn collision items] Parameters read successfully...");
        return ReturnCode::SUCCESS;
    }

} // namespace spawn_collision_items
