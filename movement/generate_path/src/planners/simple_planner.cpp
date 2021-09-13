#include "generate_path/planners/simple_planner.hpp"

namespace generate_path
{
    SimplePlanner::SimplePlanner(const rclcpp::Logger &logger)
        : IPlanner(logger)
    {
    }

    ReturnCode SimplePlanner::solve(const PathPlanningInput &path_planning_input, std::vector<ArmConfiguration> &out_path)
    {
        helpers::Timer timer("[SimplePlanner]: " + std::string(__func__), _logger);
        auto real_vector_state_space = std::make_shared<ompl::base::RealVectorStateSpace>(path_planning_input.limits.size());

        // Joints constraints
        ompl::base::RealVectorBounds bounds(path_planning_input.limits.size());
        for (size_t i = 0; i < path_planning_input.limits.size(); ++i)
        {
            bounds.setLow(i, path_planning_input.limits[i].lower);
            bounds.setHigh(i, path_planning_input.limits[i].upper);
        }
        real_vector_state_space->setBounds(bounds);

        // Space information
        auto space_information = std::make_shared<ompl::base::SpaceInformation>(real_vector_state_space);

        // Simple Setup
        auto simple_setup = std::make_shared<ompl::geometric::SimpleSetup>(space_information);

        // State validity checker
        auto state_validity_fn = [this, &path_planning_input](const ompl::base::State *state) -> bool
        {
            std::vector<double> arm_configuration;
            for (size_t i = 0; i < path_planning_input.num_dof; i++)
                arm_configuration.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);

            path_planning_input.physics_client_handler->setJointStates(arm_configuration);
            return !path_planning_input.physics_client_handler->inCollision(path_planning_input.obstacles);
        };
        simple_setup->setStateValidityChecker(state_validity_fn);

        // Start and goal states
        ompl::base::ScopedState<> start(real_vector_state_space);
        for (size_t i = 0; i < path_planning_input.start_state.size(); ++i)
            start[i] = path_planning_input.start_state[i];
        simple_setup->setStartState(start);

        auto goal_states = std::make_shared<ompl::base::GoalStates>(simple_setup->getSpaceInformation());
        for (auto goal_conf_it = path_planning_input.goal_states.begin(); goal_conf_it != path_planning_input.goal_states.end(); goal_conf_it++)
        {
            ompl::base::ScopedState<> goal(real_vector_state_space);
            for (size_t i = 0; i < goal_conf_it->size(); ++i)
                goal[i] = (*goal_conf_it)[i];
            goal_states->addState(goal);
        }
        simple_setup->setGoal(goal_states);

        // Planner
        auto planner = std::make_shared<ompl::geometric::RRT>(space_information);
        simple_setup->setPlanner(planner);

        // Solving a problem
        simple_setup->setup();

        // ///////////////////////////////////////////////////////////////////
        // // Uncomment 4 below lines to see configuration but it increase execution time significantly
        // // print the settings for this space
        // space_information->printSettings(std::cout);
        // // print the problem settings
        // simple_setup->print(std::cout);
        // ///////////////////////////////////////////////////////////////////

        // Setup planning termination condition
        auto ptc_exact_solution = ompl::base::exactSolnPlannerTerminationCondition(simple_setup->getProblemDefinition());
        auto ptc_timeout = ompl::base::timedPlannerTerminationCondition(path_planning_input.timeout.count());
        auto ptc = ompl::base::plannerOrTerminationCondition(ptc_exact_solution, ptc_timeout);

        RCLCPP_INFO_STREAM(_logger, "[Linear planner]: Solving for " << path_planning_input.timeout.count() << " seconds");
        ompl::base::PlannerStatus status = simple_setup->solve(ptc);
        if (status == ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION)
        {
            simple_setup->simplifySolution();

            ompl::geometric::PathGeometric &path = simple_setup->getSolutionPath();
            path.interpolate();

            // /////////////////////////////////////////////////////
            // path.print(std::cout);
            // /////////////////////////////////////////////////////

            // Convert OMPL path to arm configuration
            out_path = _convertOmplStatesToArmConfigurations(path, path_planning_input.num_dof);
        }
        else
            return ReturnCode::FAILURE;

        return ReturnCode::SUCCESS;
    }

} // namespace generate_path
