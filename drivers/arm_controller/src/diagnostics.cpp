#include "arm_controller/diagnostics.hpp"

Diagnostics::Diagnostics(int argc, char **argv) : BaseController(argc, argv) {}

int Diagnostics::saveDiagnostics()
{
    for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    {
        if (_arm_status.joints[jnt_idx].current_error == 21)
            continue;
        if (_arm_status.joints[jnt_idx].velocity < 0)
            continue;
        // int index = std::floor(std::fmod(std::fmod(_avg_pos[jnt_idx], (2. * M_PI)) + (2. * M_PI), (2. * M_PI)) / (2. * M_PI) * (double)_diag_samples);
        int index = std::floor((_arm_status.joints[jnt_idx].position + M_PI) * ((double)_diag_samples) / (2 * M_PI));
        // int index = std::floor((_arm_status.joints[jnt_idx].position + 0.5 * M_PI)  * ((double)_diag_samples)/ M_PI);

        _diag_data[jnt_idx].velocity[index] += _arm_status.joints[jnt_idx].velocity;
        _diag_data[jnt_idx].position[index] += 1;
        _diag_data[jnt_idx].temperature[index] += _avg_temp[jnt_idx];
    }

    return 0;
}

int Diagnostics::getAverageArmState()
{
    for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    {

        // crimge
        if (std::abs(_arm_status.joints[jnt_idx].position - _prev_pos[jnt_idx]) > 10)
        {
            _avg_pos[jnt_idx] = 0;
            for (size_t i = 0; i < _avg_samples; i++)
            {
                _avg_pos_b[jnt_idx][(i + _loop_it) % _avg_samples] = _arm_status.joints[jnt_idx].position - ((double)(_avg_samples - i) / _trajectory_rate * _avg_vel[jnt_idx]);
                _avg_pos[jnt_idx] += _avg_pos_b[jnt_idx][(i + _loop_it) % _avg_samples];
            }
            _avg_pos[jnt_idx] /= _avg_samples;
            // prev_pos[jnt_idx] = _arm_status.joints[jnt_idx].position;
            _prev_pos[jnt_idx] = _avg_pos[jnt_idx] + _avg_vel[jnt_idx] / _trajectory_rate;
        }

        _avg_pos_b[jnt_idx][_loop_it % _avg_samples] = _arm_status.joints[jnt_idx].position;
        _avg_temp_b[jnt_idx][_loop_it % _avg_samples] = _arm_status.joints[jnt_idx].temperature;
        _avg_tau_b[jnt_idx][_loop_it % _avg_samples] = _arm_status.joints[jnt_idx].torque;

        _avg_pos[jnt_idx] = 0;
        _avg_temp[jnt_idx] = 0;
        _avg_tau[jnt_idx] = 0;

        for (size_t i = 0; i < _avg_samples; i++)
        {
            _avg_pos[jnt_idx] += _avg_pos_b[jnt_idx][i];
            _avg_temp[jnt_idx] += _avg_temp_b[jnt_idx][i];
            _avg_tau[jnt_idx] += _avg_tau_b[jnt_idx][i];
        }

        _avg_pos[jnt_idx] /= _avg_samples;
        _avg_temp[jnt_idx] /= _avg_samples;
        _avg_tau[jnt_idx] /= _avg_samples;

        _avg_vel[jnt_idx] = ((_avg_pos[jnt_idx] - _prev_pos[jnt_idx]) * _trajectory_rate);
        _prev_pos[jnt_idx] = _avg_pos[jnt_idx];
        // _avg_vel[jnt_idx] = _arm_status.joints[jnt_idx].velocity;
    }
    return 0;
}

int Diagnostics::writeDiagnostics()
{
    for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    {
        if (_arm_status.joints[jnt_idx].current_error == 21)
            continue;
        if (_diag_data[jnt_idx].position.size() == 0)
            continue;
        std::string filename = std::string("/diag_j") + std::to_string(jnt_idx);
        int file_i = 0;
        while (std::filesystem::exists(std::filesystem::path(_config_path + std::string("/diag") + filename + std::string("_") + std::to_string(file_i) + std::string(".txt"))))
            file_i++;
        std::cout << _config_path + std::string("/diag") + filename + std::string("_") + std::to_string(file_i) + std::string(".txt") << std::endl;
        std::ofstream chart(_config_path + std::string("/diag") + filename + std::string("_") + std::to_string(file_i) + std::string(".txt"));
        chart.precision(6);
        for (size_t j = 0; j < _diag_data[jnt_idx].position.size(); j++)
        {
            chart << std::to_string(j / (double)_diag_samples * 2 * M_PI) << " " << std::to_string(_diag_data[jnt_idx].velocity[j] / _diag_data[jnt_idx].position[j]) << " " << std::to_string(_diag_data[jnt_idx].temperature[j] / _diag_data[jnt_idx].position[j]) << std::endl;
        }
        chart.flush();
        chart.close();
    }
    return 0;
}
int Diagnostics::communicate()
{

    for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    {
        // monitor data
        _arm_joint_state_msg.position[jnt_idx] = _arm_status.joints[jnt_idx].position;
        _arm_joint_state_msg.velocity[jnt_idx] = _arm_status.joints[jnt_idx].velocity;
        _arm_joint_state_msg.effort[jnt_idx] = _arm_status.joints[jnt_idx].torque;
        _arm_joint_state_msg.header.stamp = rclcpp::Clock().now();
    }

    // comms
    std_msgs::msg::Int32 state_msg;
    state_msg.set__data(_controller_state);
    _arm_joint_states_pub->publish(_arm_joint_state_msg);
    _controller_state_pub->publish(state_msg);

    return 0;
}

void Diagnostics::controlLoop()
{
    // GET TIME
    _time_accumulator += std::chrono::duration_cast<std::chrono::microseconds>((std::chrono::steady_clock::now() - _t_current) * _time_factor);
    _t_current = std::chrono::steady_clock::now();
    if ((_t_current - _t_start) > std::chrono::minutes(2))
    {
        for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
        {
            _arm_command.joints[jnt_idx].c_torque = 0;
            // _arm_command.joints[jnt_idx].c_status = 2;
        }
        communicate();
        _arm_command.timestamp = std::chrono::steady_clock::now();
        setArmCommand();
        writeDiagnostics();
        RCLCPP_INFO(_node->get_logger(), "shutting down");
        rclcpp::shutdown();
        exit(0);
    }
    if (!rclcpp::ok())
    {
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

    // GET JOINT STATES
    getArmStatus();

    // if (std::chrono::duration_cast<std::chrono::microseconds>((std::chrono::steady_clock::now() - _arm_status.timestamp)).count() > (1000000 / _trajectory_rate))
    // {
    //     RCLCPP_WARN(_node->get_logger(), "Communication delay exceeded loop period by %i us.", std::chrono::duration_cast<std::chrono::microseconds>((std::chrono::steady_clock::now() - _arm_status.timestamp)).count());
    // }
    // getAverageArmState();
    if ((_t_current - _t_start) > std::chrono::seconds(2))
        saveDiagnostics();
    handleControllerState(_controller_state);

    for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    {
        _arm_command.joints[jnt_idx].c_torque = _const_torque;
        // _arm_command.joints[jnt_idx].c_status = 3;
    }
    _arm_command.timestamp = std::chrono::steady_clock::now();
    setArmCommand();
    // execute callbacks
    //  _exec->spin_some(std::chrono::nanoseconds(1000));

    _loop_it++;



    // int barWidth = 70;
    // float progress = std::chrono::duration_cast<std::chrono::seconds>(_t_current - _t_start).count() / 120.;

    // std::cout << "[";
    // int pos = barWidth * progress;
    // for (int i = 0; i < barWidth; ++i) {
    //     if (i < pos) std::cout << "=";
    //     else if (i == pos) std::cout << ">";
    //     else std::cout << " ";
    // }
    // std::cout << "] " << int(progress * 100.0) << " %\r";
    // std::cout.flush();



    // control loop frequency
    //  _remaining_time = std::floor(1000000 / _trajectory_rate - std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - _t_current).count());

    // if (_remaining_time < 0)
    // {
    //     RCLCPP_ERROR(_node->get_logger(), "Loop taking too long to execute. Loop took: %i us.", std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - _t_current).count());
    // }
    // std::this_thread::sleep_for(std::chrono::microseconds(_remaining_time));
}

int Diagnostics::jointInit()
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
    _joints_number = 6;

    getArmStatus();

    for (size_t i = 0; i < _joints_number; i++)
    {
        if (_arm_status.joints[i].current_error)
        {
            RCLCPP_ERROR_STREAM(_node->get_logger(), "Current joint error: " << _arm_status.joints[i].current_error << ", previous joint error: " << _arm_status.joints[i].prev_error << " on joint " << i);
        }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(_node->get_logger(), "Joints number: %i", _joints_number);

    RCLCPP_INFO(_node->get_logger(), "Done initializing joints.");
    return 0;
}

int Diagnostics::idInit()
{
    return 0;
}

int Diagnostics::paramInit()
{
    // PARAMETERS INIT
    RCLCPP_INFO(_node->get_logger(), "Initializing controller parameters.");
    _node->declare_parameter<std::string>("config_path", "");
    _node->declare_parameter<double>("loop_frequency", 500.);
    _node->declare_parameter<double>("communication_rate", 100.);
    _node->declare_parameter<double>("tq", 12.);

    _node->get_parameter("config_path", _config_path);
    _node->get_parameter("loop_frequency", _trajectory_rate);
    _node->get_parameter("communication_rate", _communication_rate);
    _node->get_parameter("tq", _const_torque);
    RCLCPP_INFO(_node->get_logger(), "Constant torque: %f.", _const_torque);
    _avg_samples = size_t(_avg_samples_t * _trajectory_rate);
    std::cout << "avg_samples: " << _avg_samples << std::endl;
    return 0;
}

int Diagnostics::varInit(size_t joints_number)
{

    RCLCPP_INFO(_node->get_logger(), "Initializing controller variables.");
    // _arm_command.joints.resize(joints_number);
    _friction_chart.resize(joints_number);
    _Kp.resize(joints_number);
    _Ki.resize(joints_number);
    _Kd.resize(joints_number);
    _FFv.resize(joints_number);
    _FFa.resize(joints_number);
    _i_clamp_h.resize(joints_number);
    _i_clamp_l.resize(joints_number);
    _c_friction_val.resize(joints_number);

    _set_joint_state_msg.name.resize(joints_number);
    _set_joint_state_msg.position.resize(joints_number);
    _set_joint_state_msg.velocity.resize(joints_number);
    _set_joint_state_msg.effort.resize(joints_number);

    _arm_joint_state_msg.name.resize(joints_number);
    _arm_joint_state_msg.position.resize(joints_number);
    _arm_joint_state_msg.velocity.resize(joints_number);
    _arm_joint_state_msg.effort.resize(joints_number);

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

    _diag_data.resize(joints_number);

    // MEASUREMENT INIT
    for (size_t i = 0; i < joints_number; i++)
    {
        _diag_data[i].position.resize(_diag_samples);
        _diag_data[i].velocity.resize(_diag_samples);
        _diag_data[i].temperature.resize(_diag_samples);
        _diag_data[i].static_f_torque.resize(_diag_samples);
        for (size_t j = 0; j < _diag_samples; j++)
        {
            _diag_data[i].position[j] = 0;
            _diag_data[i].velocity[j] = 0;
            _diag_data[i].temperature[j] = 0;
            _diag_data[i].static_f_torque[j] = 0;
        }

        _frick_acu[i] = 0;
        _prev_pos[i] = 0;
        _avg_acc_b[i].resize(_avg_samples);
        _avg_pos_b[i].resize(_avg_samples);
        _avg_vel_b[i].resize(_avg_samples);
        _avg_temp_b[i].resize(_avg_samples);
        _avg_tau_b[i].resize(_avg_samples);
        for (size_t j = 0; j < _avg_samples; j++)
        {
            _avg_acc_b[i][j] = 0;
            _avg_pos_b[i][j] = 0;
            _avg_vel_b[i][j] = 0;
            _avg_temp_b[i][j] = 0;
            _avg_tau_b[i][j] = 0;
        }
    }

    // GET STARTING POSITION - HOLD TRAJECTORY
    getArmStatus();
    _trajectory.points.resize(1);
    _trajectory.points[0].positions.resize(joints_number);
    _trajectory.points[0].velocities.resize(joints_number);
    _trajectory.points[0].accelerations.resize(joints_number);
    for (size_t i = 0; i < joints_number; i++)
    {
        _trajectory.points[0].positions[i] = _arm_status.joints[i].position;
        _trajectory.points[0].velocities[i] = 0.;
        _trajectory.points[0].accelerations[i] = 0.;
        _trajectory.points[0].time_from_start.sec = 0.;
        _trajectory.points[0].time_from_start.nanosec = 0.;
    }

    // TIME INIT
    _loop_it = 0;
    _t_start = std::chrono::steady_clock::now();
    _time_accumulator = std::chrono::microseconds(0);
    _controller_state = STOP;
    _time_factor = 1.;
    _prev_time_factor = 1.;
    _t_current = std::chrono::steady_clock::now();
    _t_stop = std::chrono::steady_clock::now();
    _slowdown_duration = std::chrono::microseconds(1000000);

    RCLCPP_INFO(_node->get_logger(), "Done initializing variables.");
    return 0;
}

void Diagnostics::init()
{
    jointInit();
    idInit();
    paramInit();
    varInit(_joints_number);
    jointPositionInit();

    // CONTROL LOOP
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    RCLCPP_INFO(_node->get_logger(), "Done initializing, entering control loop");

    rclcpp::TimerBase::SharedPtr loop_timer, comms_timer;
    comms_timer = _node->create_wall_timer(std::chrono::microseconds(int(1000000 / _communication_rate)), std::bind(&Diagnostics::communicate, this), _cb_group);
    loop_timer = _node->create_wall_timer(std::chrono::microseconds(int(1000000 / _trajectory_rate)), std::bind(&Diagnostics::controlLoop, this), _cb_group);
    _exec->spin();
}

int main(int argc, char **argv)
{
    Diagnostics controller(argc, argv);
    controller.init();
    return 0;
}