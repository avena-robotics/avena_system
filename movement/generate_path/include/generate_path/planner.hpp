#ifndef GENERATE_PATH__PLANNER_HPP_
#define GENERATE_PATH__PLANNER_HPP_

// // ___Avena___
// #include <bullet_client/b3RobotSimulatorClientAPI.h>

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
  class Planner
  {
  public:
    Planner();
    virtual ~Planner() = default;
    virtual ReturnCode solve(const PathPlanningInput &path_planning_input, std::vector<ArmConfiguration> &out_path);
  
  private:
    int _calculateContactPointsAmount(const PathPlanningInput &path_planning_input);

  };

} // namespace generate_path

#endif // GENERATE_PATH__PLANNER_HPP_
