#include "generate_path/planners/base_planner.hpp"

namespace generate_path
{
    IPlanner::IPlanner(const rclcpp::Logger &logger)
        : _logger(logger)
    {
    }

    std::vector<ArmConfiguration> IPlanner::_convertOmplStatesToArmConfigurations(ompl::geometric::PathGeometric &ompl_path, size_t num_dof)
    {
        std::vector<ArmConfiguration> out_path;
        std::vector<ompl::base::State *> &states = ompl_path.getStates();
        out_path.resize(states.size());
        for (size_t i = 0; i < states.size(); ++i)
        {
            auto state = states[i]->as<ompl::base::State>();
            ArmConfiguration path_configuration(num_dof);
            for (size_t joint_idx = 0; joint_idx < path_configuration.size(); ++joint_idx)
                path_configuration[joint_idx] = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[joint_idx];
            out_path[i] = path_configuration;
        }
        return out_path;
    }

} // namespace generate_path
