#ifndef SPAWN_COLLISION_ITEMS__COMMONS_HPP_
#define SPAWN_COLLISION_ITEMS__COMMONS_HPP_

// ___CPP___
#include <map>
#include <memory>

// ___Other___
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nlohmann/json.hpp>

// ___ROS___
#include <rclcpp/rclcpp.hpp>

// ___Avena___
#include <physics_client_handler/physics_client_handler.hpp>
#include <helpers_commons/helpers_commons.hpp>
#include <helpers_vision/helpers_vision.hpp>

// ___Package___
#include "spawn_collision_items/visibility_control.h"

namespace spawn_collision_items
{
  // static const rclcpp::Logger LOGGER = rclcpp::get_logger("spawn_collision_items");
  using json = nlohmann::json;
  using Handle = int;
  constexpr int InvalidHandle = -128;

  enum class ReturnCode
  {
    SUCCESS,
    FAILURE,
  };

  enum class ShapeType
  {
    BOX = 0,
    SPHERE = 1,
    CYLINDER = 2,
  };

  struct ItemElement
  {
    int32_t id = 0;
    int32_t item_id = -1;
    std::string label;
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_ptcld;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cam1_ptcld;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cam2_ptcld;
    std::vector<json> parts_description;
  };

  struct Item
  {
    int32_t id = 0;
    std::string item_id_hash{};
    std::string label{};
    Eigen::Affine3f pose;
    std::vector<ItemElement> item_elements;
  };

  struct PartAdditionalData
  {
    Handle item_handle;
    std::string item_label;
  };

  class SpawnCollisionItemsError : public std::exception
  {
  public:
    explicit SpawnCollisionItemsError(const std::string &error)
    {
      _error = "[Spawn collision items]: " + error;
    }

    const char *what() const noexcept override
    {
      return _error.c_str();
    }

  private:
    std::string _error;
  };

} // namespace spawn_collision_items

#endif // SPAWN_COLLISION_ITEMS__COMMONS_HPP_
