#ifndef GENERATE_PATH__BASE_PLANNER_HPP_
#define GENERATE_PATH__BASE_PLANNER_HPP_

// ___CPP___
#include <memory>

// ___OMPL___
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/base/goals/GoalStates.h>

// ___Package___
#include "generate_path/commons.hpp"

namespace generate_path
{
  namespace ob = ompl::base;
  namespace og = ompl::geometric;

  class IPlanner
  {
  public:
    IPlanner(const rclcpp::Logger &logger);
    virtual ~IPlanner() = default;
    virtual ReturnCode solve(const PathPlanningInput &path_planning_input, std::vector<ArmConfiguration> &out_path) = 0;

    using SharedPtr = std::shared_ptr<IPlanner>;
    using UniquePtr = std::unique_ptr<IPlanner>;

  protected:
    PathPlanningInput _path_planning_input;
    rclcpp::Logger _logger;
    virtual std::vector<ArmConfiguration> _convertOmplStatesToArmConfigurations(ompl::geometric::PathGeometric &ompl_path, size_t num_dof);
  };

} // namespace generate_path

#endif // GENERATE_PATH__BASE_PLANNER_HPP_
