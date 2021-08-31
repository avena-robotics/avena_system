#include "inverse_kinematics/inverse_kinematics.hpp"

namespace inverse_kinematics
{

    InverseKinematics::InverseKinematics(physics_client_handler::PhysicsClientHandler::SharedPtr physics_server_handler,
                                         helpers::commons::RobotInfo &robot_info,
                                         const rclcpp::Logger &logger)
        : _physics_server_handler(physics_server_handler),
          _robot_info(robot_info),
          _logger(logger)
    {
        if (!_physics_server_handler)
            throw IkError("Physics client handler is not initialized");

        // Get transform to base link
        if (auto robot_base_tf = helpers::vision::getTransformAffine("world", _robot_info.base_link_name))
        {
            _reference_frame = robot_base_tf->cast<double>();
            RCLCPP_INFO(_logger, "[IK]: Using \"world\" as reference frame for end effector pose");
        }
        else
            throw IkError("Cannot read transform for robot base");
    }

    ArmConfiguration InverseKinematics::computeIk(const Eigen::Affine3d &end_effector_pose, bool in_robot_base_frame)
    {
        auto arm_configuration = computeIk(end_effector_pose, _robot_info.soft_limits, in_robot_base_frame);
        return arm_configuration;
    }

    ArmConfiguration InverseKinematics::computeIk(const Eigen::Affine3d &end_effector_pose,
                                                  const std::vector<Limits> &joint_limits,
                                                  bool in_robot_base_frame)
    {
        helpers::Timer timer(__func__, _logger);

        auto current_state = _physics_server_handler->getJointStates();

        // Read collision from physics server using physics client handler
        physics_client_handler::Obstacles collision_objects;
        {
            helpers::Timer timer("Getting collision objects handles", _logger);
            collision_objects = _physics_server_handler->getCollisionObjectsHandles();
        }

        ArmConfiguration goal_configuration;

        // Solving with IK Fast
        RCLCPP_INFO(_logger, "[IK]: Solving with IK Fast");
        goal_configuration = _solveWithIkFast(end_effector_pose, joint_limits, collision_objects, in_robot_base_frame);

        if (goal_configuration.size() == 0)
        {
            // Solving with Bullet
            RCLCPP_INFO(_logger, "[IK]: Cannot solve with IK Fast. Solving IK with Bullet");
            _physics_server_handler->setJointStates(current_state);
            goal_configuration = _physics_server_handler->computeIk(end_effector_pose, joint_limits, collision_objects);
        }
        else
            RCLCPP_INFO(_logger, "Found IK solution with IK Fast");

        _physics_server_handler->setJointStates(current_state);
        return goal_configuration;
    }

    ArmConfiguration InverseKinematics::_solveWithIkFast(const Eigen::Affine3d &end_effector_pose,
                                                         const std::vector<Limits> &joint_limits,
                                                         const physics_client_handler::Obstacles &collision_objects,
                                                         bool in_robot_base_frame)
    {
        // Get end effector transform relative to robot base
        Eigen::Affine3d end_effector_pose_robot_frame;
        if (!in_robot_base_frame)
            end_effector_pose_robot_frame = _reference_frame.inverse() * end_effector_pose;

        // Get rotation of end effector
        IkReal eerot[9];
        auto quat = Eigen::Quaterniond(end_effector_pose_robot_frame.rotation());
        auto rot_matrix = quat.normalized().toRotationMatrix();
        eerot[0] = rot_matrix(0);
        eerot[1] = rot_matrix(3);
        eerot[2] = rot_matrix(6);
        eerot[3] = rot_matrix(1);
        eerot[4] = rot_matrix(4);
        eerot[5] = rot_matrix(7);
        eerot[6] = rot_matrix(2);
        eerot[7] = rot_matrix(5);
        eerot[8] = rot_matrix(8);

        // Get translation of end effector
        IkReal eetrans[3];
        auto trans_vec = Eigen::Vector3d(end_effector_pose_robot_frame.translation());
        eetrans[0] = trans_vec(0); // - _reference_frame.transform.translation.x;
        eetrans[1] = trans_vec(1); // - _reference_frame.transform.translation.y;
        eetrans[2] = trans_vec(2); // - _reference_frame.transform.translation.z;

        ArmConfiguration goal_configuration;
        if (_robot_info.robot_name == "avena")
        {
            // Avena
            IkSolutionList<IkReal> solutions;
            if (ik_avena::ComputeIk(eetrans, eerot, NULL, solutions))
            {
                RCLCPP_DEBUG_STREAM(_logger, "[IK]: Found " << solutions.GetNumSolutions() << " IK solutions");
                std::vector<IkReal> solvalues(ik_avena::GetNumJoints());
                for (std::size_t i = 0; i < solutions.GetNumSolutions(); ++i)
                {
                    RCLCPP_DEBUG_STREAM(_logger, "[IK]: Validating solution " << i + 1 << " / " << solutions.GetNumSolutions());

                    const IkSolutionBase<IkReal> &sol = solutions.GetSolution(i);
                    std::vector<IkReal> vsolfree(sol.GetFree().size());
                    sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);

                    // ///////////////////////////////////////////////////////////////////////
                    // // Single configuration visualization
                    // _setJointStates(solvalues);
                    // cv::Mat img = cv::Mat::ones(100, 100, CV_8UC1) * 255;
                    // cv::putText(img,                                                                       //target image
                    //             std::to_string(i + 1) + "_" + std::to_string(solutions.GetNumSolutions()), //text
                    //             cv::Point(10, img.rows / 2),                                               //top-left position
                    //             cv::FONT_HERSHEY_DUPLEX,
                    //             0.5,
                    //             CV_RGB(118, 185, 0), //font color
                    //             2);
                    // cv::imshow("kek", img);
                    // cv::waitKey();
                    // printf("\n");
                    // ///////////////////////////////////////////////////////////////////////

                    // Validate calculated configuration
                    if (_validateArmConfiguration(end_effector_pose, solvalues, joint_limits, collision_objects) != ReturnCode::SUCCESS)
                    {
                        RCLCPP_DEBUG(_logger, "[IK]: Checking another configuration...");
                        continue;
                    }

                    goal_configuration = solvalues;
                    break;
                }
            }
            else
                RCLCPP_DEBUG(_logger, "[IK]: Inverse kinematics engine cannot find any solution");
        }
        else if (_robot_info.robot_name == "franka")
        {
            // Franka
            const auto free_joint_idx = ik_franka::GetFreeIndices();
            const double joint_state_increment = 0.1;
            const auto free_joint_limits = joint_limits[free_joint_idx[0]];
            const double middle_range = (free_joint_limits.lower + free_joint_limits.upper) / 2;
            double lower_joint_state = middle_range;
            double upper_joint_state = middle_range + joint_state_increment;

            size_t amount_of_samples = 0;
            auto check_single_ik = [=, &amount_of_samples](const double &free_joint_val) mutable -> ArmConfiguration
            {
                ArmConfiguration goal_configuration;

                std::vector<IkReal> vfree = {free_joint_val};
                IkSolutionList<IkReal> solutions;
                if (ik_franka::ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions))
                {
                    RCLCPP_DEBUG_STREAM(_logger, "[IK]: Found " << solutions.GetNumSolutions() << " IK solutions");
                    std::vector<IkReal> solvalues(ik_franka::GetNumJoints());
                    for (std::size_t i = 0; i < solutions.GetNumSolutions(); ++i)
                    {
                        RCLCPP_DEBUG_STREAM(_logger, "[IK]: Validating solution " << i + 1 << " / " << solutions.GetNumSolutions());
                        amount_of_samples++;

                        const IkSolutionBase<IkReal> &sol = solutions.GetSolution(i);
                        std::vector<IkReal> vsolfree(sol.GetFree().size());
                        sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);

                        // Validate calculated configuration
                        if (_validateArmConfiguration(end_effector_pose, solvalues, joint_limits, collision_objects) != ReturnCode::SUCCESS)
                        {
                            RCLCPP_DEBUG(_logger, "[IK]: Checking another configuration...");
                            continue;
                        }

                        goal_configuration = solvalues;

                        // cv::Mat img = cv::Mat::ones(100, 100, CV_8UC1) * 255;
                        // cv::putText(img,                                                                       //target image
                        //             std::to_string(i + 1) + "_" + std::to_string(solutions.GetNumSolutions()), //text
                        //             cv::Point(10, img.rows / 2),                                               //top-left position
                        //             cv::FONT_HERSHEY_DUPLEX,
                        //             0.5,
                        //             CV_RGB(118, 185, 0), //font color
                        //             2);
                        // cv::imshow("kek", img);
                        // cv::waitKey();
                        // printf("\n");
                        break;
                    }
                }
                return goal_configuration;
            };

            while (true)
            {
                if (lower_joint_state < free_joint_limits.lower && upper_joint_state > free_joint_limits.upper)
                    break;

                if (lower_joint_state >= free_joint_limits.lower)
                {
                    goal_configuration = check_single_ik(lower_joint_state);
                    if (goal_configuration.size() > 0)
                        break;
                }

                if (upper_joint_state <= free_joint_limits.upper)
                {
                    goal_configuration = check_single_ik(upper_joint_state);
                    if (goal_configuration.size() > 0)
                        break;
                }

                lower_joint_state -= joint_state_increment;
                upper_joint_state += joint_state_increment;
            }

            // RCLCPP_WARN_STREAM(_logger, amount_of_samples);
        }
        else
        {
            throw IkError("Unsupported robot to calculate IK");
        }

        return goal_configuration;
    }

    ReturnCode InverseKinematics::_validateArmConfiguration(const Eigen::Affine3d &end_effector_pose,
                                                            const ArmConfiguration &joint_state,
                                                            const std::vector<Limits> &joint_limits,
                                                            const std::vector<int> &obstacles)
    {
        if (_checkJointLimits(joint_state, joint_limits) != ReturnCode::SUCCESS)
        {
            RCLCPP_DEBUG(_logger, "[IK]: Invalid final state. Joint states are outside of limits");
            return ReturnCode::FAILURE;
        }

        _physics_server_handler->setJointStates(joint_state);
        if (auto ee_pose_opt = _physics_server_handler->getEndEffectorPose())
        {
            if (_validateEndEffectorPose(end_effector_pose, *ee_pose_opt) != ReturnCode::SUCCESS)
            {
                RCLCPP_DEBUG(_logger, "[IK]: End effector for calculated configuration is in wrong pose");
                return ReturnCode::FAILURE;
            }
        }
        else
        {
            RCLCPP_DEBUG(_logger, "[IK]: Cannot read end effector from physics server");
            return ReturnCode::FAILURE;
        }

        if (_physics_server_handler->inCollision(obstacles))
        {
            RCLCPP_DEBUG(_logger, "[IK]: Invalid final state. Robot is in collision with scene or itself");
            return ReturnCode::FAILURE;
        }
        return ReturnCode::SUCCESS;
    }

    ReturnCode InverseKinematics::_checkJointLimits(const ArmConfiguration &joint_states, const std::vector<Limits> &joint_limits)
    {
        RCLCPP_DEBUG(_logger, "[IK]: Check whether joint states are in limits");
        if (joint_states.size() != joint_limits.size())
        {
            RCLCPP_WARN_STREAM(_logger, "[IK]: Amount of joint states (" << joint_states.size() << ") is different than amount of joint limits (" << joint_limits.size() << ")");
            return ReturnCode::FAILURE;
        }
        auto error_code = ReturnCode::SUCCESS;
        for (size_t i = 0; i < joint_states.size(); ++i)
        {
            if (joint_states[i] < joint_limits[i].lower || joint_states[i] > joint_limits[i].upper)
            {
                RCLCPP_DEBUG_STREAM(_logger, "[IK]: Joint: " << i + 1 << ": value: " << joint_states[i] << ", limits: (" << joint_limits[i].lower << ", " << joint_limits[i].upper << ") [outside of limits]");
                error_code = ReturnCode::FAILURE;
            }
            else
                RCLCPP_DEBUG_STREAM(_logger, "[IK]: Joint: " << i + 1 << ": value: " << joint_states[i] << ", limits: (" << joint_limits[i].lower << ", " << joint_limits[i].upper << ")");
        }
        return error_code;
    }

    ReturnCode InverseKinematics::_validateEndEffectorPose(const Eigen::Affine3d &end_effector_pose,
                                                           const Eigen::Affine3d &calculated_end_effector_pose)
    {
        auto aff_diff = helpers::commons::getDiffBetweenAffines(end_effector_pose, calculated_end_effector_pose);
        bool close_enough = (aff_diff[0] < POSITION_THRESHOLD) &&
                            std::abs(aff_diff[1]) < ORIENTATION_THRESHOLD &&
                            std::abs(aff_diff[2]) < ORIENTATION_THRESHOLD &&
                            std::abs(aff_diff[3]) < ORIENTATION_THRESHOLD;
        return close_enough ? ReturnCode::SUCCESS : ReturnCode::FAILURE;
    }

    // void InverseKinematics::_updateJointLimits(const float &joint_range_coeff)
    // {
    //     // const double range_tighten_coeff = 0.9; // values from 0.0 - 1.0 how much scale down range for joint limits
    //     for (size_t i = 0; i < _robot_info.limits.size(); ++i)
    //     {
    //         auto range_middle = (_robot_info.limits[i].lower + _robot_info.limits[i].upper) / 2.0;
    //         auto range_middle_to_limit_dist = _robot_info.limits[i].upper - range_middle;
    //         auto offset = (range_middle_to_limit_dist * (1.0 - joint_range_coeff)) / 2.0;
    //         _robot_info.soft_limits[i].lower += _robot_info.soft_limits[i].lower + offset;
    //         _robot_info.soft_limits[i].upper -= _robot_info.soft_limits[i].upper - offset;
    //     }
    // }

} // namespace inverse_kinematics
