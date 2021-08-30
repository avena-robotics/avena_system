#include "generate_path/planner.hpp"

namespace generate_path
{
    Planner::Planner()
    {
    }

    ReturnCode Planner::solve(const PathPlanningInput &path_planning_input, std::vector<ArmConfiguration> &out_path)
    {
        helpers::Timer timer(__func__, LOGGER);
        auto space = std::make_shared<ompl::base::RealVectorStateSpace>(path_planning_input.limits.size());

        // Joints constraints
        ompl::base::RealVectorBounds bounds(path_planning_input.limits.size());
        for (size_t i = 0; i < path_planning_input.limits.size(); ++i)
        {
            bounds.setLow(i, path_planning_input.limits[i].lower);
            bounds.setHigh(i, path_planning_input.limits[i].upper);
        }
        space->setBounds(bounds);

        auto si = std::make_shared<ompl::base::SpaceInformation>(space);

        si->setStateValidityChecker([=](const ompl::base::State *state)
                                    {
                                        std::vector<double> arm_configuration;
                                        for (size_t i = 0; i < path_planning_input.state_space_size; i++)
                                            arm_configuration.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
                                        
                                        path_planning_input.physics_client_handler->setJointStates(arm_configuration);
                                        return !path_planning_input.physics_client_handler->inCollision();
                                    });

        ompl::base::ScopedState<> start(space);
        for (size_t i = 0; i < path_planning_input.start_state.size(); ++i)
            start[i] = path_planning_input.start_state[i];

        ompl::base::ScopedState<> goal(space);
        for (size_t i = 0; i < path_planning_input.goal_state.size(); ++i)
            goal[i] = path_planning_input.goal_state[i];

        auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
        pdef->setStartAndGoalStates(start, goal);

        auto planner = std::make_shared<ompl::geometric::RRT>(si);

        planner->setProblemDefinition(pdef);
        planner->setIntermediateStates(true);

        // perform setup steps for the planner
        planner->setup();

        // print the settings for this space
        si->printSettings(std::cout);

        // print the problem settings
        pdef->print(std::cout);

        // attempt to solve the problem within one second of planning time
        ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(1.0);

        if (solved)
        {
            ompl::geometric::PathGeometric ompl_path(dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
            const std::vector<ompl::base::State *> &states = ompl_path.getStates();
            ompl::base::State *state;
            out_path.resize(states.size());
            for (size_t i = 0; i < states.size(); ++i)
            {
                state = states[i]->as<ompl::base::State>();
                ArmConfiguration path_configuration(path_planning_input.state_space_size);
                for (size_t joint_idx = 0; joint_idx < path_configuration.size(); ++joint_idx)
                    path_configuration[joint_idx] = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[joint_idx];
                out_path[i] = path_configuration;
            }
        }
        else
            return ReturnCode::FAILURE;

        return ReturnCode::SUCCESS;
    }

} // namespace generate_path
