#include "arm_controller/single_joint_debug.hpp"

SingleJointDebug::SingleJointDebug(int argc, char **argv) : BaseController(argc, argv) {}
SingleJointDebug::~SingleJointDebug()
{

    _run_data.erase(_run_data.begin(), _run_data.begin() + time_steps);
    _stop_data.erase(_stop_data.begin(), _stop_data.begin() +  stop_time);

    std::sort(_run_data.begin(), _run_data.end());
    std::sort(_stop_data.begin(), _stop_data.end());


    // double cutoff = 0.001;
    // _run_data.erase(_run_data.begin(), _run_data.begin() + (int)std::floor(cutoff * _run_data.size()));
    // _run_data.erase(_run_data.end() - (int)std::floor(cutoff * _run_data.size()), _run_data.end());

    // _stop_data.erase(_stop_data.begin(), _stop_data.begin() + (int)std::floor(cutoff * _stop_data.size()));
    // _stop_data.erase(_stop_data.end() - (int)std::floor(cutoff * _stop_data.size()), _stop_data.end());

    _run_mse = get_mse(_run_data);
    _run_max = get_max(_run_data);
    _run_mean = get_mean(_run_data);


    _stop_mse = get_mse(_stop_data);
    _stop_max = get_max(_stop_data);
    _stop_mean = get_mean(_stop_data);

    std::cout << "\tRUN: \nMean square average: " << _run_mse << "\nMaximum: " << _run_max << "\nMean: " << _run_mean << "(" << std::sqrt(_run_mse) << ")\n\tSTOP: \nMean square average: " << _stop_mse << "\nMaximum: " << _stop_max << "\nMean: " << _stop_mean << "(" << std::sqrt(_stop_mse) << ")" << std::endl;

    *_stat_file << "\tRUN: \nMean square average: " << _run_mse << "\nMaximum: " << _run_max << "\nMean: " << _run_mean << "(" << std::sqrt(_run_mse) << ")\n\tSTOP: \nMean square average: " << _stop_mse << "\nMaximum: " << _stop_max << "\nMean: " << _stop_mean << "(" << std::sqrt(_stop_mse) << ")" << std::endl;
    _stat_file->flush();
    _stat_file->close();
}
int SingleJointDebug::jointInit()
{
    // JOINT COMMUNICATION INIT
    RCLCPP_INFO(_node->get_logger(), "Starting executor");
    _exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    _exec->add_node(_node);
    RCLCPP_INFO(_node->get_logger(), "Getting arm state from CANDRIVER...");
    getArmStatus();

    // while (_arm_status.joints.size() < 6)
    // {
    //     RCLCPP_ERROR(_node->get_logger(), "Received invalid arm state. Waiting ...");
    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    //     getArmStatus();
    // }

    RCLCPP_INFO(_node->get_logger(), "Got arm state from CANDRIVER");
    // int connected_joints = 0;
    // for (size_t i = 0; i < _joints_number; i++)
    // {
    //     if (_arm_status.joints[i].state != 420)
    //         connected_joints++;
    // }
    // _joints_number = connected_joints;
    _joints_number = 1;

    getArmStatus();

    for (size_t i = 0; i < _joints_number; i++)
    {
        if (_arm_status.joints[i].current_error || _arm_status.joints[i].current_error == 21)
        {
            RCLCPP_ERROR_STREAM(_node->get_logger(), "Current joint error: " << _arm_status.joints[i].current_error << ", previous joint error: " << _arm_status.joints[i].prev_error << " on joint " << i);
        }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(_node->get_logger(), "Joints number: %i", _joints_number);

    RCLCPP_INFO(_node->get_logger(), "Done initializing joints.");
    return 0;
}

int SingleJointDebug::varInit(size_t joints_number)
{

    RCLCPP_INFO(_node->get_logger(), "Initializing controller variables.");
    // _arm_command.joints.resize(joints_number);
    _friction_chart.resize(joints_number);
    _error.resize(joints_number);
    for (int i = 0; i < _error.size(); i++)
    {
        _error[i] = 0;
    }

    _set_joint_state_msg.name.resize(joints_number);
    _set_joint_state_msg.position.resize(joints_number);
    _set_joint_state_msg.velocity.resize(joints_number);
    _set_joint_state_msg.effort.resize(joints_number);

    _arm_joint_state_msg.name.resize(joints_number);
    _arm_joint_state_msg.position.resize(joints_number);
    _arm_joint_state_msg.velocity.resize(joints_number);
    _arm_joint_state_msg.effort.resize(joints_number);

    _arm_joint_errors_msg.position.resize(joints_number);
    _arm_joint_errors_msg.velocity.resize(joints_number);
    _arm_joint_errors_msg.effort.resize(joints_number);

    _q.resize(joints_number);
    _qd.resize(joints_number);
    _qdd.resize(joints_number);

    _avg_acc.resize(joints_number);
    _avg_pos.resize(joints_number);
    _avg_vel.resize(joints_number);
    _avg_temp.resize(joints_number);
    _avg_tau.resize(joints_number);
    _prev_pos.resize(joints_number);

    _avg_acc_b.resize(joints_number);
    _avg_pos_b.resize(joints_number);
    _avg_vel_b.resize(joints_number);
    _avg_temp_b.resize(joints_number);
    _avg_tau_b.resize(joints_number);
    _frick_acu.resize(joints_number);

    // GET SINUSOIDAL TRAJECTORY
    getArmStatus();

    time_steps = std::floor(_trajectory_period * _trajectory_rate);
    stop_time = std::floor(_trajectory_period / 8 * _trajectory_rate);

    _trajectory.points.resize(time_steps + stop_time * 2);
    for (int i = 0; i < (time_steps / 4); i++)
    {
        _trajectory.points[i].positions.resize(joints_number);
        _trajectory.points[i].velocities.resize(joints_number);
        _trajectory.points[i].accelerations.resize(joints_number);
        for (size_t jnt_idx = 0; jnt_idx < joints_number; jnt_idx++)
        {
            _trajectory.points[i].positions[jnt_idx] = std::sin(double(i) / double(time_steps) * 2 * M_PI) * _sin_amp;
            _trajectory.points[i].velocities[jnt_idx] = std::cos(double(i) / double(time_steps) * 2 * M_PI) * _sin_amp * 2 * M_PI / _trajectory_period;
            _trajectory.points[i].accelerations[jnt_idx] = 0.;
            _trajectory.points[i].time_from_start.sec = 0.;
            _trajectory.points[i].time_from_start.nanosec = 0.;
        }
    }

    for (int i = (time_steps / 4); i < (time_steps / 4 + stop_time); i++)
    {
        _trajectory.points[i].positions.resize(joints_number);
        _trajectory.points[i].velocities.resize(joints_number);
        _trajectory.points[i].accelerations.resize(joints_number);
        for (size_t jnt_idx = 0; jnt_idx < joints_number; jnt_idx++)
        {
            _trajectory.points[i].positions[jnt_idx] = std::sin(double(time_steps / 4) / double(time_steps) * 2 * M_PI) * _sin_amp;
            _trajectory.points[i].velocities[jnt_idx] = 0;
            _trajectory.points[i].accelerations[jnt_idx] = 0.;
            _trajectory.points[i].time_from_start.sec = 0.;
            _trajectory.points[i].time_from_start.nanosec = 0.;
        }
    }

    for (int i = (time_steps / 4 + stop_time); i < (time_steps * 3 / 4 + stop_time); i++)
    {
        _trajectory.points[i].positions.resize(joints_number);
        _trajectory.points[i].velocities.resize(joints_number);
        _trajectory.points[i].accelerations.resize(joints_number);
        for (size_t jnt_idx = 0; jnt_idx < joints_number; jnt_idx++)
        {
            _trajectory.points[i].positions[jnt_idx] = std::sin(double(i - stop_time) / double(time_steps) * 2 * M_PI) * _sin_amp;
            _trajectory.points[i].velocities[jnt_idx] = std::cos(double(i - stop_time) / double(time_steps) * 2 * M_PI) * _sin_amp * 2 * M_PI / _trajectory_period;
            _trajectory.points[i].accelerations[jnt_idx] = 0.;
            _trajectory.points[i].time_from_start.sec = 0.;
            _trajectory.points[i].time_from_start.nanosec = 0.;
        }
    }

    for (int i = time_steps * 3 / 4 + stop_time; i < time_steps * 3 / 4 + stop_time * 2; i++)
    {
        _trajectory.points[i].positions.resize(joints_number);
        _trajectory.points[i].velocities.resize(joints_number);
        _trajectory.points[i].accelerations.resize(joints_number);
        for (size_t jnt_idx = 0; jnt_idx < joints_number; jnt_idx++)
        {
            _trajectory.points[i].positions[jnt_idx] = std::sin(double(time_steps * 3 / 4) / double(time_steps) * 2 * M_PI) * _sin_amp;
            _trajectory.points[i].velocities[jnt_idx] = 0;
            _trajectory.points[i].accelerations[jnt_idx] = 0.;
            _trajectory.points[i].time_from_start.sec = 0.;
            _trajectory.points[i].time_from_start.nanosec = 0.;
        }
    }

    for (int i = time_steps * 3 / 4 + stop_time * 2; i < time_steps + stop_time * 2; i++)
    {
        _trajectory.points[i].positions.resize(joints_number);
        _trajectory.points[i].velocities.resize(joints_number);
        _trajectory.points[i].accelerations.resize(joints_number);
        for (size_t jnt_idx = 0; jnt_idx < joints_number; jnt_idx++)
        {
            _trajectory.points[i].positions[jnt_idx] = std::sin(double(i - stop_time * 2) / double(time_steps) * 2 * M_PI) * _sin_amp;
            _trajectory.points[i].velocities[jnt_idx] = std::cos(double(i - stop_time * 2) / double(time_steps) * 2 * M_PI) * _sin_amp * 2 * M_PI / _trajectory_period;
            _trajectory.points[i].accelerations[jnt_idx] = 0.;
            _trajectory.points[i].time_from_start.sec = 0.;
            _trajectory.points[i].time_from_start.nanosec = 0.;
        }
    }

    _mass_x = 0.66;
    _mass = 1.8+20.;

    // TIME INIT
    //  _loop_it = std::floor(std::asin(_arm_status.joints[6].position/_sin_amp));
    _loop_it = 0;
    RCLCPP_INFO(_node->get_logger(), "Starting point: %f", std::sin(_loop_it));
    _t_start = std::chrono::steady_clock::now();
    _time_accumulator = std::chrono::microseconds(0);
    _controller_state = EXECUTE;
    _time_factor = 1.;
    _prev_time_factor = 1.;
    _t_current = std::chrono::steady_clock::now();
    _t_stop = std::chrono::steady_clock::now();
    _slowdown_duration = std::chrono::microseconds(1000000);

    _stat_file = std::make_shared<std::ofstream>(_config_path + std::string("/precision_statistics") + std::string(".txt"));
    _stat_file->precision(6);

    RCLCPP_INFO(_node->get_logger(), "Done initializing variables.");
    return 0;
}

int SingleJointDebug::paramInit()
{
    // PARAMETERS INIT
    RCLCPP_INFO(_node->get_logger(), "Initializing controller parameters.");
    _node->declare_parameter<std::string>("config_path", "");
    _node->declare_parameter<double>("loop_frequency", 500.);
    _node->declare_parameter<double>("communication_rate", 100.);
    _node->declare_parameter<double>("trajectory_period", 24.);
    _node->declare_parameter<double>("sin_amp", 2.8);
    _node->declare_parameter<double>("set_vel", 0.1);
    _node->declare_parameter<double>("avg_samples", 0.005);

    _node->get_parameter("config_path", _config_path);
    _node->get_parameter("loop_frequency", _trajectory_rate);
    _node->get_parameter("communication_rate", _communication_rate);
    _node->get_parameter("trajectory_period", _trajectory_period);
    _node->get_parameter("sin_amp", _sin_amp);
    _node->get_parameter("set_vel", _set_vel);
    _node->get_parameter("avg_samples", _avg_samples_t);

    _avg_samples = size_t(_avg_samples_t * _trajectory_rate);
    std::cout << "avg_samples: " << _avg_samples << std::endl;

    _Kp.resize(_joints_number);
    _Ki.resize(_joints_number);
    _Kd.resize(_joints_number);
    _FFv.resize(_joints_number);
    _FFa.resize(_joints_number);
    _i_clamp_h.resize(_joints_number);
    _i_clamp_l.resize(_joints_number);
    _c_friction_val.resize(_joints_number);

    for (size_t i = 0; i < _joints_number; i++)
    {
        // set defaults in case config file is not provided
        _node->declare_parameter<double>("Kp_gain_" + std::to_string(i), 38);
        _node->declare_parameter<double>("Ki_gain_" + std::to_string(i), 1);
        _node->declare_parameter<double>("Kd_gain_" + std::to_string(i), 25);
        _node->declare_parameter<double>("FFv_gain_" + std::to_string(i), 17);
        _node->declare_parameter<double>("FFa_gain_" + std::to_string(i), 1);
        _node->declare_parameter<double>("Coulomb_friction_" + std::to_string(i), 9);
        _node->declare_parameter<double>("i_clamp_h_" + std::to_string(i), 20);
        _node->declare_parameter<double>("i_clamp_l_" + std::to_string(i), -20);
        // get parameter values from config file
        _node->get_parameter("Kp_gain_" + std::to_string(i), _Kp[i]);
        _node->get_parameter("Ki_gain_" + std::to_string(i), _Ki[i]);
        _node->get_parameter("Kd_gain_" + std::to_string(i), _Kd[i]);
        _node->get_parameter("FFv_gain_" + std::to_string(i), _FFv[i]);
        _node->get_parameter("FFa_gain_" + std::to_string(i), _FFv[i]);
        _node->get_parameter("Coulomb_friction_" + std::to_string(i), _c_friction_val[i]);
        _node->get_parameter("i_clamp_h_" + std::to_string(i), _i_clamp_h[i]);
        _node->get_parameter("i_clamp_l_" + std::to_string(i), _i_clamp_l[i]);
        // initialize PIDs
        _pid_ctrl.push_back(PID(_Kp[i], _Ki[i], _Kd[i], 1.0 / _trajectory_rate, _i_clamp_l[i], _i_clamp_h[i], _avg_samples_t * _trajectory_rate));
    }
    RCLCPP_INFO(_node->get_logger(), "Done setting PIDs");

    // FRICTION INIT
    loadFrictionCoeffs(_config_path + std::string("/friction/friction_coeffs_"));
    RCLCPP_INFO(_node->get_logger(), "Loaded friction coefficients");
    return 0;
}

int SingleJointDebug::communicate()
{

    for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    {
        // monitor data
        _arm_joint_state_msg.position[jnt_idx] = _arm_status.joints[jnt_idx].position;
        _arm_joint_state_msg.velocity[jnt_idx] = _arm_status.joints[jnt_idx].velocity;
        _arm_joint_state_msg.effort[jnt_idx] = _arm_status.joints[jnt_idx].torque;
        _arm_joint_state_msg.header.stamp = rclcpp::Clock().now();

        _set_joint_state_msg.position[jnt_idx] = _trajectory.points[_loop_it].positions[jnt_idx];
        _set_joint_state_msg.velocity[jnt_idx] = (_pid_ctrl[jnt_idx].getValue(_error[jnt_idx])*1.2/(std::abs(_arm_status.joints[jnt_idx].torque)+1.))*100.;
        _set_joint_state_msg.effort[jnt_idx] = _arm_command.joints[jnt_idx].c_torque;
        _set_joint_state_msg.header.stamp = rclcpp::Clock().now();

        _arm_joint_errors_msg.position[jnt_idx] = _error[jnt_idx];
        _arm_joint_errors_msg.velocity[jnt_idx] = _mass_x * _mass * sin(_arm_status.joints[jnt_idx].position)*9.81 + compensateFriction_coeffs(_trajectory.points[_loop_it].velocities[jnt_idx], friction_coefficients[jnt_idx])*1.2;
        _arm_joint_errors_msg.effort[jnt_idx] = _pid_ctrl[jnt_idx].getValue(_error[jnt_idx])*1.2;
        _arm_joint_errors_msg.header.stamp = rclcpp::Clock().now();
    }

    // comms
    std_msgs::msg::Int32 state_msg;
    state_msg.set__data(_controller_state);
    _arm_joint_errors_pub->publish(_arm_joint_errors_msg);
    _set_joint_states_pub->publish(_set_joint_state_msg);
    _arm_joint_states_pub->publish(_arm_joint_state_msg);
    // _controller_state_pub->publish(state_msg);

    return 0;
}

double SingleJointDebug::get_mse(std::vector<double> data)
{
    double acc = 0;
    for (size_t i = 0; i < data.size(); i++)
    {
        acc += std::pow(data[i], 2);
    }
    return acc / (double)data.size();
}

double SingleJointDebug::get_mean(std::vector<double> data)
{
    double acc = 0;
    for (size_t i = 0; i < data.size(); i++)
    {
        acc += data[i];
    }
    return acc / (double)data.size();
}

double SingleJointDebug::get_max(std::vector<double> data)
{
    double temp = 0;
    for (size_t i = 0; i < data.size(); i++)
    {
        if (std::abs(data[i]) > temp)
            temp = std::abs(data[i]);
    }
    return temp;
}
void SingleJointDebug::controlLoop()
{
    // GET TIME
    //  _time_accumulator += std::chrono::duration_cast<std::chrono::microseconds>((std::chrono::steady_clock::now() - _t_current) * _time_factor);
    //  _t_current = std::chrono::steady_clock::now();

    if (!rclcpp::ok())
    {
        // std::sort(_run_data.begin(), _run_data.end());
        // std::sort(_stop_data.begin(), _stop_data.end());
        // double cutoff = 0.05;
        // _run_data.erase(_run_data.begin(), _run_data.begin() + (int)std::floor(cutoff * _run_data.size()));
        // _run_data.erase(_run_data.end() - (int)std::floor(cutoff * _run_data.size()), _run_data.end());

        // _stop_data.erase(_stop_data.begin(), _stop_data.begin() + (int)std::floor(cutoff * _stop_data.size()));
        // _stop_data.erase(_stop_data.end() - (int)std::floor(cutoff * _stop_data.size()), _stop_data.end());

        // _run_mse = get_mse(_run_data);
        // _run_max = get_max(_run_data);
        // _run_mean = get_mean(_run_data);

        // _stop_mse = get_mse(_stop_data);
        // _stop_max = get_max(_stop_data);
        // _stop_mean = get_mean(_stop_data);

        // *_stat_file << "\tRUN: \nMean square average: " << _run_mse << "\nMaximum: " << _run_max << "\nMean: " << _run_mean << "\n\tSTOP: \nMean square average: " << _stop_mse << "\nMaximum: " << _stop_max << "\nMean: " << _stop_mean << std::endl;
        // _stat_file->flush();
        // _stat_file->close();

        for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
        {
            _arm_command.joints[jnt_idx].c_torque = 0;
            // _arm_command.joints[jnt_idx].c_status = 2;
        }
        communicate();
        _arm_command.timestamp = std::chrono::steady_clock::now();
        setArmCommand();
        RCLCPP_INFO(_node->get_logger(), "shutting down");
        rclcpp::shutdown();
        exit(0);
    }
    double temp = _arm_status.joints[0].position;
    // GET JOINT STATES
    getArmStatus();

    // handleControllerState(_controller_state);

    // if (_loop_it >= _trajectory.points.size())
    // {
    //     _loop_it = 0;
    // }
    // for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    // {
    //     if (_arm_status.joints[jnt_idx].state == 420)
    //         continue;
    //     updateParams(_pid_ctrl, jnt_idx);
    //     _error[jnt_idx] = _trajectory.points[_loop_it].positions[jnt_idx] - (_arm_status.joints[jnt_idx].position);
    //     _arm_command.joints[jnt_idx].c_torque = _pid_ctrl[jnt_idx].getValue(_error[jnt_idx]);
    //     if (_arm_command.joints[jnt_idx].c_torque > 20.)
    //         _arm_command.joints[jnt_idx].c_torque = 20.;
    //     if (_arm_command.joints[jnt_idx].c_torque < -20.)
    //         _arm_command.joints[jnt_idx].c_torque = -20.;

    //     _arm_command.joints[jnt_idx].c_status = 3;
    // }

    // MIN MOVEMENT

    // for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    // {
    //     // if (_arm_status.joints[jnt_idx].state == 420)
    //     //     continue;
    //     if ((std::chrono::steady_clock::now() - _t_start) > std::chrono::seconds(1))
    //     {
    //         if (std::abs(temp - _arm_status.joints[jnt_idx].position) < 0.00005)
    //         {
    //             _arm_command.joints[jnt_idx].c_torque = 15.;
    //             _arm_command.joints[jnt_idx].c_status = 3;
    //         }
    //         else
    //         {
    //             p_q = temp;
    //             std::cout << "td: " << std::chrono::duration_cast<std::chrono::milliseconds>((std::chrono::steady_clock::now() - _t_start) - std::chrono::seconds(1)).count() << std::endl;
    //             p_f = 1;
    //             _arm_command.joints[jnt_idx].c_torque = 0.;
    //             _arm_command.joints[jnt_idx].c_status = 3.;
    //             _t_start = std::chrono::steady_clock::now();
    //         }
    //     }
    //     else
    //     {
    //         // if ((std::chrono::steady_clock::now() - _t_start) < std::chrono::milliseconds(10))
    //         // {
    //         //     _arm_command.joints[jnt_idx].c_torque = 19.;
    //         //     _arm_command.joints[jnt_idx].c_status = 3;
    //         // }
    //         // else
    //         {
    //             if ((std::chrono::steady_clock::now() - _t_start) > std::chrono::milliseconds(500) && p_f)
    //             {
    //                 std::cout << "qd: " << _arm_status.joints[jnt_idx].position - p_q << std::endl;
    //                 p_f = 0;
    //             }
    //             _arm_command.joints[jnt_idx].c_torque = 0.;
    //             _arm_command.joints[jnt_idx].c_status = 3.;
    //         }
    //     }
    // }

    //*************
    // if ((std::chrono::steady_clock::now() - _t_start) > std::chrono::seconds(1))
    //     _t_start = std::chrono::steady_clock::now();
    // _node->get_parameter("set_vel", _set_vel);
    // for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    // {
    //     updateParams(_pid_ctrl, jnt_idx);
    //     _error[jnt_idx] = _set_vel - _arm_status.joints[jnt_idx].velocity;
    //     _arm_command.joints[jnt_idx].c_torque = _pid_ctrl[jnt_idx].getValue(_error[jnt_idx]);
    //     _arm_command.joints[jnt_idx].c_status = 3;
    // }

    // SINE
    if (_loop_it >= _trajectory.points.size() - 1)
    {
        _loop_it = 0;
    }
    for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    {
        //  if (_arm_status.joints[jnt_idx].state == 420)
        //      continue;
        updateParams(_pid_ctrl, jnt_idx);
        _error[jnt_idx] = _trajectory.points[_loop_it].positions[jnt_idx] - (_arm_status.joints[jnt_idx].position);

        if (_loop_it > 0 && _loop_it < (time_steps / 4))
        {
            _run_data.push_back(_error[jnt_idx]);
        }

        if (_loop_it > (time_steps / 4) && _loop_it < (time_steps / 4 + stop_time))
        {
            _stop_data.push_back(_error[jnt_idx]);
        }

        if (_loop_it > (time_steps / 4 + stop_time) && _loop_it < (time_steps * 3 / 4 + stop_time))
        {
            _run_data.push_back(_error[jnt_idx]);
        }

        if (_loop_it > (time_steps * 3 / 4 + stop_time) && _loop_it < time_steps * 3 / 4 + stop_time * 2)
        {
            _stop_data.push_back(_error[jnt_idx]);
        }

        if (_loop_it > time_steps * 3 / 4 + stop_time * 2 && _loop_it < time_steps + stop_time * 2)
        {
            _run_data.push_back(_error[jnt_idx]);
        }

        double grav_comp = _mass_x * _mass * sin(_arm_status.joints[jnt_idx].position)*9.81;
        _arm_command.joints[jnt_idx].c_torque = (_pid_ctrl[jnt_idx].getValue(_error[jnt_idx]) + compensateFriction_coeffs(_trajectory.points[_loop_it].velocities[jnt_idx], friction_coefficients[jnt_idx]) + grav_comp)*1.2;
        //  _arm_command.joints[jnt_idx].c_torque = _pid_ctrl[jnt_idx].getValue(_error[jnt_idx]);
        //  _arm_command.joints[jnt_idx].c_torque =grav_comp;

        if (_arm_command.joints[jnt_idx].c_torque > 300.)
            _arm_command.joints[jnt_idx].c_torque = 300.;
        if (_arm_command.joints[jnt_idx].c_torque < -300.)
            _arm_command.joints[jnt_idx].c_torque = -300.;

        // _arm_command.joints[jnt_idx].c_status = 3;
        // _error[jnt_idx] = _trajectory.points[((_loop_it-int(0.01*_trajectory_rate))%_trajectory.points.size())].positions[jnt_idx] - (_arm_status.joints[jnt_idx].position);
    }

    _arm_command.timestamp = std::chrono::steady_clock::now();
    setArmCommand();

    _loop_it++;
}

void SingleJointDebug::init()
{
    jointInit();
    paramInit();
    varInit(_joints_number);
    jointPositionInit();

    // CONTROL LOOP
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    RCLCPP_INFO(_node->get_logger(), "Done initializing, entering control loop");

    rclcpp::TimerBase::SharedPtr loop_timer, comms_timer;
    comms_timer = _node->create_wall_timer(std::chrono::microseconds(int(1000000 / _communication_rate)), std::bind(&SingleJointDebug::communicate, this), _cb_group);
    loop_timer = _node->create_wall_timer(std::chrono::microseconds(int(1000000 / _trajectory_rate)), std::bind(&SingleJointDebug::controlLoop, this), _cb_group);
    _exec->spin();
}

int main(int argc, char **argv)
{
    SingleJointDebug controller(argc, argv);
    controller.init();
    return 0;
}