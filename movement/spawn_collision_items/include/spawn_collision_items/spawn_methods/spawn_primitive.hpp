#ifndef SPAWN_COLLISION_ITEMS__SPAWN_PRIMITIVE_HPP_
#define SPAWN_COLLISION_ITEMS__SPAWN_PRIMITIVE_HPP_

#include "spawn_collision_items/spawn_methods/base_spawn_method.hpp"

namespace spawn_collision_items
{
  class SpawnPrimitive : public ISpawnMethod
  {
  public:
    SpawnPrimitive(const rclcpp::Logger &logger);
    virtual ~SpawnPrimitive();
    virtual Handle spawn(const json &part_description, const PartAdditionalData &additional_data) override;

  private:
    std::vector<float> _assignObjectSizes(const json &part_description);
  };
}

#endif // SPAWN_COLLISION_ITEMS__SPAWN_PRIMITIVE_HPP_
