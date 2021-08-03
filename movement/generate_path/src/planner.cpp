#include "generate_path/planner.hpp"

namespace generate_path
{
    Planner::Planner()
    {
    }

    ReturnCode Planner::solve(const PathPlanningInput &path_planning_input, std::vector<ArmConfiguration> &out_path)
    {
        helpers::Timer timer(__func__, LOGGER);
        auto space = std::make_shared<ompl::base::RealVectorStateSpace>(path_planning_input.scene_info->joint_handles.size());

        // Joints constraints
        ompl::base::RealVectorBounds bounds(path_planning_input.scene_info->joint_handles.size());
        for (size_t i = 0; i < path_planning_input.constraints->limits.size(); ++i)
        {
            bounds.setLow(i, path_planning_input.constraints->limits[i].lower);
            bounds.setHigh(i, path_planning_input.constraints->limits[i].upper);
        }
        space->setBounds(bounds);

        auto si = std::make_shared<ompl::base::SpaceInformation>(space);

        si->setStateValidityChecker([=](const ompl::base::State *state)
                                    {
                                        for (size_t i = 0; i < path_planning_input.scene_info->joint_handles.size(); i++)
                                            path_planning_input.scene_info->bullet_client->resetJointState(path_planning_input.scene_info->robot_idx, i, state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);

                                        return Planner::calculateContactPointsAmount(path_planning_input) <= path_planning_input.constraints->contact_number_allowed;
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
                ArmConfiguration path_configuration(path_planning_input.scene_info->joint_handles.size());
                for (size_t joint_idx = 0; joint_idx < path_configuration.size(); ++joint_idx)
                    path_configuration[joint_idx] = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[joint_idx];
                out_path[i] = path_configuration;
            }
        }
        else
            return ReturnCode::FAILURE;

        return ReturnCode::SUCCESS;
    }

    int Planner::calculateContactPointsAmount(const PathPlanningInput &path_planning_input)
    {
        int contacts_amount = 0;
        // Check collision with obstacles
        for (auto obstacle_idx : path_planning_input.constraints->obstacles)
        {
            b3RobotSimulatorGetContactPointsArgs obtacles_collision_args;
            obtacles_collision_args.m_bodyUniqueIdA = path_planning_input.scene_info->robot_idx;
            obtacles_collision_args.m_bodyUniqueIdB = obstacle_idx;
            b3ContactInformation contact_info;
            path_planning_input.scene_info->bullet_client->getClosestPoints(obtacles_collision_args, path_planning_input.constraints->safety_distance, &contact_info);
            contacts_amount += contact_info.m_numContactPoints;
        }

        // Check self collision
        b3ContactInformation contact_info;
        b3RobotSimulatorGetContactPointsArgs self_collision_args;
        self_collision_args.m_bodyUniqueIdA = path_planning_input.scene_info->robot_idx;
        self_collision_args.m_bodyUniqueIdB = path_planning_input.scene_info->robot_idx;
        for (int i = 0; i < path_planning_input.scene_info->bullet_client->getNumJoints(path_planning_input.scene_info->robot_idx) - 2; i++)
        {
            self_collision_args.m_linkIndexA = i;
            for (int j = i + 2; j < path_planning_input.scene_info->bullet_client->getNumJoints(path_planning_input.scene_info->robot_idx); j++)
            {
                self_collision_args.m_linkIndexB = j;
                path_planning_input.scene_info->bullet_client->getClosestPoints(self_collision_args, path_planning_input.constraints->safety_distance, &contact_info);
                contacts_amount += contact_info.m_numContactPoints;
            }
        }
        return contacts_amount;
    }

} // namespace generate_path
