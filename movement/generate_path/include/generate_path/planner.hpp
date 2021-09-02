#ifndef GENERATE_PATH__PLANNER_HPP_
#define GENERATE_PATH__PLANNER_HPP_

// ___OMPL___
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

// ___Package___
#include "generate_path/commons.hpp"

namespace generate_path
{
  namespace ob = ompl::base;
  namespace og = ompl::geometric;

  class Planner
  {
  public:

    Planner(const rclcpp::Logger &logger);
    virtual ~Planner() = default;
    virtual ReturnCode solve(const PathPlanningInput &path_planning_input, std::vector<ArmConfiguration> &out_path);
  
  private:
    rclcpp::Logger _logger;
  };

} // namespace generate_path

#endif // GENERATE_PATH__PLANNER_HPP_
