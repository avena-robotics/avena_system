#ifndef GENERATE_PATH__SIMPLE_PLANNER_HPP_
#define GENERATE_PATH__SIMPLE_PLANNER_HPP_

// ___Package___
#include "generate_path/planners/base_planner.hpp"

namespace generate_path
{
  class SimplePlanner : public IPlanner
  {
  public:
    SimplePlanner(const rclcpp::Logger &logger);
    virtual ~SimplePlanner() = default;
    virtual ReturnCode solve(const PathPlanningInput &path_planning_input, std::vector<ArmConfiguration> &out_path) override;
  
  };

} // namespace generate_path

#endif // GENERATE_PATH__SIMPLE_PLANNER_HPP_
