#ifndef GENERATE_PATH__FACTORY_PLANNER_HPP_
#define GENERATE_PATH__FACTORY_PLANNER_HPP_

// ___Package___
#include "generate_path/planners/base_planner.hpp"
#include "generate_path/planners/simple_planner.hpp"
#include "generate_path/planners/linear_planner.hpp"

namespace generate_path
{
  class FactoryPlanner
  {
  public:
    FactoryPlanner() = delete;
    virtual ~FactoryPlanner() = delete;
    static IPlanner::SharedPtr createPlanner(const rclcpp::Logger &logger, uint8_t type)
    {
      if (type == EndEffectorPose::PATH)
        return std::make_shared<SimplePlanner>(logger);
      else if (type == EndEffectorPose::LINEAR)
        return std::make_shared<LinearPlanner>(logger);
      else
        return nullptr;
    }
  };

} // namespace generate_path

#endif // GENERATE_PATH__FACTORY_PLANNER_HPP_
