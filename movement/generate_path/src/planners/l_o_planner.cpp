#include "generate_path/planners/l_o_planner.hpp"

namespace generate_path
{
    LOPlanner::LOPlanner(const rclcpp::Logger &logger)
        : IPlanner(logger)
    {
    }

    std::vector<ArmConfiguration> LOPlanner::_convertOmplStatesToArmConfigurations(ompl::geometric::PathGeometric &ompl_path, size_t num_dof)
    {
        std::vector<ompl::base::State *> &states = ompl_path.getStates();
        std::vector<ArmConfiguration> out_path(states.size());
        for (size_t i = 0; i < states.size(); ++i)
        {
            auto state = states[i]->as<ompl::base::State>();
            ArmConfiguration path_configuration(num_dof);
            for (size_t joint_idx = 0; joint_idx < path_configuration.size(); ++joint_idx)
            {
                auto x = state->as<ob::ConstrainedStateSpace::StateType>()->getState()->as<ob::RealVectorStateSpace::StateType>();
                path_configuration[joint_idx] = x->values[joint_idx];
            }
            out_path[i] = path_configuration;
        }
        return out_path;
    }

    ReturnCode LOPlanner::solve(const PathPlanningInput &path_planning_input, std::vector<ArmConfiguration> &out_path)
    {
        helpers::Timer timer("[LOPlanner]: " + std::string(__func__), _logger);

        // Ambient state space
        auto real_vector_state_space = std::make_shared<ompl::base::RealVectorStateSpace>(path_planning_input.limits.size());

        // State space bounds
        ompl::base::RealVectorBounds bounds(path_planning_input.limits.size());
        for (size_t i = 0; i < path_planning_input.limits.size(); ++i)
        {
            bounds.setLow(i, path_planning_input.limits[i].lower);
            bounds.setHigh(i, path_planning_input.limits[i].upper);
        }
        real_vector_state_space->setBounds(bounds);

        // Constraint

        std::vector<ompl::base::ConstraintPtr> constraints_list;
        
        constraints_list.push_back(std::make_shared<LinearPathConstraint>(path_planning_input, _logger));
        constraints_list.push_back(std::make_shared<OrientationConstraint>(path_planning_input, _logger));
        std::shared_ptr<ompl::base::ConstraintIntersection> constraints=std::make_shared<ompl::base::ConstraintIntersection>(path_planning_input.num_dof,constraints_list);
        constraints -> setMaxIterations(ompl::magic::CONSTRAINT_PROJECTION_MAX_ITERATIONS);

        // Combine the ambient space and the constraint into a constrained state space.
        auto constrained_state_space = std::make_shared<ompl::base::ProjectedStateSpace>(real_vector_state_space, constraints);

        // Define the constrained space information for this constrained state space.
        auto constrained_space_information = std::make_shared<ompl::base::ConstrainedSpaceInformation>(constrained_state_space);

        // Simple Setup
        auto simple_setup = std::make_shared<ompl::geometric::SimpleSetup>(constrained_space_information);

        // State validity checker
        auto state_validity_fn = [this, &path_planning_input](const ompl::base::State *state) -> bool
        {
            const Eigen::Map<Eigen::VectorXd> &x = *state->as<ompl::base::ConstrainedStateSpace::StateType>();
            std::vector<double> arm_configuration(x.data(), x.data() + x.size());

            // Check joints limits because I am not sure whether sampling
            // in constrained state space respects limits (probably it does but
            // this checks are not time consuming)
            for (size_t i = 0; i < arm_configuration.size(); i++)
            {
                if (arm_configuration[i] < path_planning_input.limits[i].lower || arm_configuration[i] > path_planning_input.limits[i].upper)
                {
                    return false;
                }
            }

            path_planning_input.physics_client_handler->setJointStates(arm_configuration);
            if (path_planning_input.physics_client_handler->inCollision(path_planning_input.obstacles))
            {
                return false;
            }

            return true;
        };
        simple_setup->setStateValidityChecker(state_validity_fn);

        ompl::base::ScopedState<> start(constrained_state_space);
        for (size_t i = 0; i < path_planning_input.start_state.size(); ++i)
            start[i] = path_planning_input.start_state[i];
        simple_setup->setStartState(start);

        auto goal_states = std::make_shared<ompl::base::GoalStates>(simple_setup->getSpaceInformation());
        for (auto goal_conf_it = path_planning_input.goal_states.begin(); goal_conf_it != path_planning_input.goal_states.end(); goal_conf_it++)
        {
            ompl::base::ScopedState<> goal(constrained_state_space);
            for (size_t i = 0; i < goal_conf_it->size(); ++i)
                goal[i] = (*goal_conf_it)[i];
            goal_states->addState(goal);
        }
        simple_setup->setGoal(goal_states);

        // Planner
        auto planner = std::make_shared<ompl::geometric::RRTstar>(constrained_space_information);
        simple_setup->setPlanner(planner);

        // Solving a problem
        simple_setup->setup();

        // std::stringstream ss;
        // ss << "\n----------------------------------------------------------" << std::endl;
        // ss << "simple_setup->print() START" << std::endl;
        // simple_setup->print(ss);
        // ss << "simple_setup->print() END" << std::endl;
        // ss << "----------------------------------------------------------" << std::endl;
        // RCLCPP_WARN(_logger, ss.str());
        // return ReturnCode::FAILURE;

        // Setup planning termination condition
        auto ptc_exact_solution = ompl::base::exactSolnPlannerTerminationCondition(simple_setup->getProblemDefinition());
        auto ptc_timeout = ompl::base::timedPlannerTerminationCondition(path_planning_input.timeout.count());
        auto ptc = ompl::base::plannerOrTerminationCondition(ptc_exact_solution, ptc_timeout);

        RCLCPP_INFO_STREAM(_logger, "[LO planner]: Solving for " << path_planning_input.timeout.count() << " seconds");
        ompl::base::PlannerStatus status = simple_setup->solve(ptc);
        if (status == ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION)
        {
            simple_setup->simplifySolution();

            ompl::geometric::PathGeometric &path = simple_setup->getSolutionPath();
            path.interpolate();

            // std::cout << "----------------------------------------------------------" << std::endl;
            // path.print(std::cout);
            // std::cout << "----------------------------------------------------------" << std::endl;

            // Convert OMPL path to arm configuration
            out_path = _convertOmplStatesToArmConfigurations(path, path_planning_input.num_dof);
        }
        else
        {
            RCLCPP_ERROR(_logger, "[LO planner]: Cannot solve for LO path planning");
            return ReturnCode::FAILURE;
        }

        return ReturnCode::SUCCESS;
    }

//     /************************
//      * LOConstraint
//      * *********************/
//     LOConstraint::LOConstraint(const PathPlanningInput &path_planning_input, const rclcpp::Logger &logger)
//         : ompl::base::Constraint(path_planning_input.num_dof, 3, 1e-3),
//           _path_planning_input(path_planning_input),
//           _logger(logger)
//     {
//         // std::cout << "Setting up constraints" << std::endl;
//         // Setup box constraint dimensions and pose
//         double dist = (path_planning_input.goal_end_effector_pose.translation() - path_planning_input.start_end_effector_pose.translation()).norm();
//         std::vector<double> dims = {0.001, 0.001, dist, 0.01, 0.01, M_PI};
//         _target_position = (path_planning_input.goal_end_effector_pose.translation() + path_planning_input.start_end_effector_pose.translation()) / 2.0;
//         Eigen::Vector3d direction_axis = (path_planning_input.goal_end_effector_pose.translation() - path_planning_input.start_end_effector_pose.translation()).normalized();
//         _target_position_orientation.setFromTwoVectors(Eigen::Vector3d::UnitZ(), direction_axis);
//         _target_orientation = Eigen::Vector3d(path_planning_input.start_end_effector_pose.rotation().eulerAngles(0, 1, 2));
//         _bounds = Bounds({-dims[0] / 2.0, -dims[1] / 2.0, -dims[2] / 2.0, -dims[3] / 2.0, -dims[4] / 2.0, -dims[5] / 2.0}, {dims[0] / 2.0, dims[1] / 2.0, dims[2] / 2.0, dims[3] / 2.0, dims[4] / 2.0, dims[5] / 2.0});
//     }

//     void LOConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
//     {
//         // std::cout << "Calculating constraints penalty" << std::endl;
//         // const Eigen::VectorXd current_values = _calcError(x);
//         const Eigen::VectorXd current_values = Eigen::VectorXd::Ones(6);
//         // std::cout << "target: " << _target_position.transpose()
//         //           << "\tasdf: " << current_values.transpose() << std::endl;
//         out = _bounds.penalty(current_values);
//         // std::cout << "asdf size: " << out.size() << std::endl
//         //           << out << std::endl;
//     }

//     void LOConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const
//     {
//         // std::cout << "Calculating constraints jacobian" << std::endl;
//         const Eigen::VectorXd constraint_error = _calcError(x);
//         // const Eigen::VectorXd constraint_error = Eigen::VectorXd::Zero(6);

//         // std::cout << "\tGetting bounds derivative" << std::endl;
//         const Eigen::VectorXd constraint_derivative = _bounds.derivative(constraint_error);
//         const Eigen::MatrixXd robot_jacobian = _robotGeometricJacobian(x).topRows(6);
//         // std::cout << "Rotating constraints jacobian" << std::endl;
//         const Eigen::MatrixXd linear_jacobian = _target_position_orientation.matrix().transpose() * robot_jacobian.topRows(3);
//         // std::cout << "Setting constraints jacobian" << std::endl;
//         // std::cout << "Bounds size: "<< _bounds.size() << std::endl;
//         for (std::size_t i = 0; i < 3; i++)
//         {
//             out.row(i) = constraint_derivative[i] * linear_jacobian.row(i);
//         }
//         for (std::size_t i = 3; i < _bounds.size(); i++)
//         {
//             out.row(i) = constraint_derivative[i] * robot_jacobian.row(i);
//         }
//     }

//     Eigen::VectorXd LOConstraint::_calcError(const Eigen::Ref<const Eigen::VectorXd> &x) const
//     {
//         // std::cout << "Getting error" << std::endl;
//         Eigen::Affine3d x_pose = _forwardKinematics(x);
//         Eigen::Vector3d e1, e2;
//         Eigen::Array<double, 1, 6> error;
//         // std::cout << "\tCalculating e1" << std::endl;
//         // std::cout<<_target_position_orientation.matrix().transpose() * (x_pose.translation() - _target_position)<<std::endl;
//         e1 = _target_position_orientation.matrix().transpose() * (x_pose.translation() - _target_position);
//         // std::cout << "\tCalculating e2" << std::endl;
//         // std::cout<<x_pose.rotation().eulerAngles(0, 1, 2) - _target_orientation<<std::endl;
//         e2 = x_pose.rotation().eulerAngles(0, 1, 2) - _target_orientation;
//         // std::cout << "\tConcatenating error" << std::endl;
//         error << e1.transpose(), e2.transpose();
//         return error.transpose();
//     }

//     // Eigen::MatrixXd LOConstraint::_calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd> &x) const
//     // {
//     //     return _target_orientation.matrix().transpose() * _robotGeometricJacobian(x).bottomRows(3);
//     // }

//     Eigen::Affine3d LOConstraint::_forwardKinematics(const Eigen::Ref<const Eigen::VectorXd> &joint_values) const
//     {
//         // helpers::Timer timer(__func__, _logger);
//         std::vector<double> current_configuration(joint_values.data(), joint_values.data() + joint_values.size());
//         return _path_planning_input.kinematics_engine->fk->computeFk(current_configuration);
//         // _path_planning_input.physics_client_handler->setJointStates(current_configuration);
//         // if (auto ee_eff_opt = _path_planning_input.physics_client_handler->getEndEffectorPose())
//         // {
//         //     return *ee_eff_opt;
//         // }
//         // throw std::runtime_error("[Orientation planner]: Cannot get end effector pose");
//     }

//     Eigen::MatrixXd LOConstraint::_robotGeometricJacobian(const Eigen::Ref<const Eigen::VectorXd> &joint_values) const
//     {
//         // std::cout << "Getting robot jacobian" << std::endl;
//         std::vector<double> current_configuration(joint_values.data(), joint_values.data() + joint_values.size());
//         if (auto jacobian_opt = _path_planning_input.physics_client_handler->getJacobian(current_configuration))
//             return *jacobian_opt;
//         throw std::runtime_error("[LO planner]: Cannot get Jacobian");
//     }

}
