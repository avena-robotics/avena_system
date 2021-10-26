#include "kinematics/inverse_kinematics.hpp"

namespace kinematics
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

    std::vector<ArmConfiguration> InverseKinematics::computeAllIk(const Eigen::Affine3d &end_effector_pose,
                                                                  const std::chrono::duration<double> &timeout,
                                                                  bool in_robot_base_frame)
    {
        auto arm_configurations = computeAllIk(end_effector_pose, _robot_info.soft_limits, timeout, in_robot_base_frame);
        return arm_configurations;
    }

    std::vector<ArmConfiguration> InverseKinematics::computeAllIk(const Eigen::Affine3d &end_effector_pose,
                                                                  const std::vector<Limits> &joint_limits,
                                                                  const std::chrono::duration<double> &timeout,
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

        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + timeout;
        auto time_expired = [=]() -> bool
        {
            // Only half of the timeout is allowed for IK Fast calculations
            return std::chrono::steady_clock::now() - start_time >= timeout / 2;
        };

        ///////////////////////////////////////////////////////////////////////
        // IK Fast solver
        // Get end effector transform relative to robot base
        Eigen::Affine3d end_effector_pose_robot_frame;
        if (!in_robot_base_frame)
            end_effector_pose_robot_frame = _reference_frame.inverse() * end_effector_pose;

        // Get rotation of end effector
        IkReal eerot[9];
        Eigen::Quaterniond quat(end_effector_pose_robot_frame.rotation());
        Eigen::Matrix3d rot_matrix = quat.normalized().toRotationMatrix();
        rot_matrix.transposeInPlace();
        for (long int i = 0; i < rot_matrix.size(); i++)
            eerot[i] = rot_matrix(i);

        // Get translation of end effector
        IkReal eetrans[3];
        Eigen::Vector3d trans_vec = Eigen::Vector3d(end_effector_pose_robot_frame.translation());
        for (long int i = 0; i < trans_vec.size(); i++)
            eetrans[i] = trans_vec(i);
        RCLCPP_DEBUG(_logger, "[IK]: Goal pose translation: x: %f y: %f z: %f",eetrans[0],eetrans[1],eetrans[2]);
        RCLCPP_DEBUG_STREAM(_logger, "[IK]: Goal pose rotation:\n"<<rot_matrix);
        std::vector<ArmConfiguration> goal_configurations;
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
                    if (time_expired())
                        break;

                    RCLCPP_DEBUG_STREAM(_logger, "[IK]: Validating solution " << i + 1 << " / " << solutions.GetNumSolutions());

                    const IkSolutionBase<IkReal> &sol = solutions.GetSolution(i);
                    std::vector<IkReal> vsolfree(sol.GetFree().size());
                    sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);

                    // Validate calculated configuration
                    if (_validateArmConfiguration(end_effector_pose, solvalues, joint_limits, collision_objects) != ReturnCode::SUCCESS)
                    {
                        RCLCPP_DEBUG(_logger, "[IK]: Checking another configuration...");
                        continue;
                    }

                    RCLCPP_DEBUG(_logger, "[IK]: Adding valid IK Fast configuration");
                    goal_configurations.emplace_back(solvalues);
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
            auto check_single_free_joint_ik = [=, &amount_of_samples](const double &free_joint_val, std::vector<ArmConfiguration> &goal_configurations) mutable -> void
            {
                std::vector<IkReal> vfree = {free_joint_val};
                IkSolutionList<IkReal> solutions;
                if (ik_franka::ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions))
                {
                    RCLCPP_DEBUG_STREAM(_logger, "[IK]: Found " << solutions.GetNumSolutions() << " IK solutions");
                    std::vector<IkReal> solvalues(ik_franka::GetNumJoints());
                    for (std::size_t i = 0; i < solutions.GetNumSolutions(); ++i)
                    {
                        if (time_expired())
                            break;

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

                        RCLCPP_DEBUG(_logger, "[IK]: Adding valid IK Fast configuration");
                        goal_configurations.emplace_back(solvalues);
                    }
                }
            };

            while (true)
            {
                if (time_expired())
                    break;

                if (lower_joint_state < free_joint_limits.lower && upper_joint_state > free_joint_limits.upper)
                    break;

                if (lower_joint_state >= free_joint_limits.lower)
                    check_single_free_joint_ik(lower_joint_state, goal_configurations);

                if (upper_joint_state <= free_joint_limits.upper)
                    check_single_free_joint_ik(upper_joint_state, goal_configurations);

                lower_joint_state -= joint_state_increment;
                upper_joint_state += joint_state_increment;
            }

            // RCLCPP_WARN_STREAM(_logger, amount_of_samples);
        }
        else
        {
            throw IkError("Unsupported robot to calculate IK");
        }
        RCLCPP_DEBUG_STREAM(_logger, "Amount of valid configurations calculated with IK Fast: " << goal_configurations.size());

        ///////////////////////////////////////////////////////////////////////
        // Bullet IK
        _physics_server_handler->setJointStates(current_state);
        auto remaining_timeout = end_time - std::chrono::steady_clock::now();
        if (remaining_timeout > std::chrono::seconds(0))
        {
            RCLCPP_DEBUG_STREAM(_logger, "Remaining timeout for Bullet IK calculation: " << std::chrono::duration_cast<std::chrono::milliseconds>(remaining_timeout).count() << " [ms]");
            auto bullet_goal_configurations = _physics_server_handler->computeAllIk(end_effector_pose, joint_limits, collision_objects, remaining_timeout);
            goal_configurations.insert(goal_configurations.end(), bullet_goal_configurations.begin(), bullet_goal_configurations.end());
        }
        ///////////////////////////////////////////////////////////////////////

        _physics_server_handler->setJointStates(current_state);
        return goal_configurations;
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
        Eigen::Quaterniond quat(end_effector_pose_robot_frame.rotation());
        Eigen::Matrix3d rot_matrix = quat.normalized().toRotationMatrix();
        rot_matrix.transposeInPlace();
        for (long int i = 0; i < rot_matrix.size(); i++)
            eerot[i] = rot_matrix(i);
        
        // Get translation of end effector
        IkReal eetrans[3];
        Eigen::Vector3d trans_vec = Eigen::Vector3d(end_effector_pose_robot_frame.translation());
        for (long int i = 0; i < trans_vec.size(); i++)
            eetrans[i] = trans_vec(i);

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

    void InverseKinematics::setReferenceFrame(const Eigen::Affine3d &reference_frame)
    {
        _reference_frame = reference_frame;
    }

} // namespace kinematics
