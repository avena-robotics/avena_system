#ifndef SPAWN_COLLISION_ITEMS__BASE_SPAWN_METHOD_HPP_
#define SPAWN_COLLISION_ITEMS__BASE_SPAWN_METHOD_HPP_

// ___CPP___
#include <map>
#include <memory>

// ___Other
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Eigen>
#include <nlohmann/json.hpp>

// ___ROS___
#include <rclcpp/rclcpp.hpp>

// ___Package___
#include "spawn_collision_items/commons.hpp"
#include "helpers_commons/helpers_commons.hpp"
#include "helpers_vision/helpers_vision.hpp"

namespace spawn_collision_items
{
  class ISpawnMethod
  {
  public:
    ISpawnMethod(const rclcpp::Logger &logger)
        : _logger(logger)
    {
    }
    virtual ~ISpawnMethod() = default;
    virtual Handle spawn(const json &part_description, const PartAdditionalData &additional_data) = 0;

  protected:
    const std::map<std::string, ShapeType_e> _shape_type{
        {"box", ShapeType_e::BOX},
        {"sphere", ShapeType_e::SPHERE},
        {"cylinder", ShapeType_e::CYLINDER},
    };
    rclcpp::Logger _logger;
  };
} // namespace spawn_collision_items

#endif // SPAWN_COLLISION_ITEMS__BASE_SPAWN_METHOD_HPP_
