#ifndef SPAWN_COLLISION_ITEMS__FACTORY_SPAWN_METHODS_HPP_
#define SPAWN_COLLISION_ITEMS__FACTORY_SPAWN_METHODS_HPP_

#include <memory>
#include "spawn_methods/base_spawn_method.hpp"
#include "spawn_methods/spawn_primitive.hpp"

namespace spawn_collision_items
{
  class FactorySpawnMethod
  {
  public:
    FactorySpawnMethod() = delete;
    virtual ~FactorySpawnMethod() = delete;
    static std::shared_ptr<ISpawnMethod> createSpawnMethod(const std::string &spawn_method, const rclcpp::Logger &logger)
    {
      if (spawn_method == "cylinder" ||
          spawn_method == "box" ||
          spawn_method == "sphere")
        return std::make_shared<SpawnPrimitive>(logger);
      // else if (spawn_method == "tool")
      //   return std::make_shared<SpawnTool>();
      else
        return nullptr;
    }
  };
}
#endif // SPAWN_COLLISION_ITEMS__FACTORY_SPAWN_METHODS_HPP_
