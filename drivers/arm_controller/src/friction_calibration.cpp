#include "arm_controller/friction_calibration.hpp"

// TODO: init essential functionalities, get ready to start
//

FrictionCalibration::FrictionCalibration(int argc, char **argv) : BaseController(argc, argv) {}

int FrictionCalibration::jointInit()
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
    int connected_joints = 0;
    for (size_t i = 0; i < _joints_number; i++)
    {
        if (_arm_status.joints[i].state != 69)
            connected_joints++;
    }
    _joints_number = connected_joints;

    for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    {
        _arm_config.joints[jnt_idx].operation_mode = 1;
        _arm_config.joints[jnt_idx].working_area_enabled = 0;
        _arm_config.joints[jnt_idx].absolute_position = 1;
    }
    _arm_config.timestamp = std::chrono::steady_clock::now();
    setArmConfig();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    getArmStatus();

    for (size_t i = 0; i < _joints_number; i++)
    {
        if (_arm_status.joints[i].state == 255 || _arm_status.joints[i].state == 69)
        {
            RCLCPP_ERROR_STREAM(_node->get_logger(), "Current joint error: " << _arm_status.joints[i].current_error << ", previous joint error: " << _arm_status.joints[i].prev_error << " on joint " << i);
        }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(_node->get_logger(), "Joints number: %i", _joints_number);

    RCLCPP_INFO(_node->get_logger(), "Done initializing joints.");
    return 0;
}

int FrictionCalibration::varInit(size_t joints_number)
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

    // _q.resize(joints_number);
    // _qd.resize(joints_number);
    // _qdd.resize(joints_number);

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

int FrictionCalibration::paramInit()
{
    // PARAMETERS INIT
    RCLCPP_INFO(_node->get_logger(), "Initializing controller parameters.");

    _node->declare_parameter<std::string>("config_path", "");
    _node->declare_parameter<double>("loop_frequency", 500.);
    _node->declare_parameter<double>("communication_rate", 100.);
    _node->declare_parameter<double>("avg_samples", 0.05);

    _node->get_parameter("config_path", _config_path);
    _node->get_parameter("avg_samples", _avg_samples_t);
    _node->get_parameter("loop_frequency", _trajectory_rate);
    _node->get_parameter("communication_rate", _communication_rate);

    _avg_samples = size_t(_avg_samples_t * _trajectory_rate);
    std::cout << "avg_samples: " << _avg_samples << std::endl;

    // fu
    //  MEASUREMENT INIT
    for (size_t i = 0; i < _joints_number; i++)
    {

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

    for (size_t i = 0; i < _joints_number; i++)
    {
        // set defaults in case config file is not provided
        _node->declare_parameter<double>("Kp_gain_" + std::to_string(i), 350);
        _node->declare_parameter<double>("Ki_gain_" + std::to_string(i), 600);
        _node->declare_parameter<double>("Kd_gain_" + std::to_string(i), 30);
        _node->declare_parameter<double>("FFv_gain_" + std::to_string(i), 0);
        _node->declare_parameter<double>("FFa_gain_" + std::to_string(i), 0);
        _node->declare_parameter<double>("Coulomb_friction_" + std::to_string(i), 0);
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

        std::cout << "PID " << i << "\nKp: " << _Kp[i] << "\nKi: " << _Ki[i] << "\nKd: " << _Kd[i] << "\ntr: " << 1.0 / _trajectory_rate << "\nsmpl: " << _avg_samples << std::endl;
        _pid_ctrl.push_back(PID(_Kp[i], _Ki[i], _Kd[i], 1.0 / _trajectory_rate, _i_clamp_l[i], _i_clamp_h[i], _avg_samples));
    }

    RCLCPP_INFO(_node->get_logger(), "Done setting PIDs");

    // FRICTION INIT
    // loadFrictionChart(_config_path + std::string("/friction_chart_"));
    // RCLCPP_INFO(_node->get_logger(), "Loaded friction chart");
    loadFrictionCoeffs(_config_path + std::string("/friction_coeffs_"));
    RCLCPP_INFO(_node->get_logger(), "Loaded friction chart");
    return 0;
}

// initialize movement functionalities, start controller
void FrictionCalibration::init()
{

    jointInit();

    varInit(_joints_number);
    paramInit();

    jointPositionInit();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    {
        _arm_state_command.joints[jnt_idx].state = 3;
    }
    _arm_state_command.timestamp = std::chrono::steady_clock::now();
    setArmState();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto calib_time = std::chrono::steady_clock::now();
    int direction = 1;

    for (int asdf = 0; asdf < 6; asdf++)
    {
        if (asdf % 2)
        {
            direction = -1;
        }
        else
        {
            direction = 1;
        }

        std::vector<double> set_vel;
        double vel_increment = 0.05 * direction, max_vel = 0.8 * direction;
        // const int _avg_samples = _avg_samples;
        std::vector<double> v_avg, prev_pos;
        std::vector<std::vector<double>> p_avg, temp_avg, tau_avg;
        std::vector<double> p_avg_s, temp_avg_s, tau_avg_s, goal_v_avg_acc;
        std::shared_ptr<std::ofstream> chart;
        friction_comp friction_point;
        std::vector<double> tq_acc;
        std::vector<int> tq_inc;
        std::vector<bool> vel_achi;
        std::vector<bool> cal_done;
        std::vector<std::chrono::steady_clock::time_point> valid_vel_time;
        std::vector<std::chrono::steady_clock::time_point> cycle_time;

        goal_v_avg_acc.resize(_joints_number);
        vel_achi.resize(_joints_number);
        cal_done.resize(_joints_number);
        valid_vel_time.resize(_joints_number);
        cycle_time.resize(_joints_number);
        tq_acc.resize(_joints_number);
        tq_inc.resize(_joints_number);
        set_vel.resize(_joints_number);
        v_avg.resize(_joints_number);
        prev_pos.resize(_joints_number);
        p_avg.resize(_joints_number);
        temp_avg.resize(_joints_number);
        tau_avg.resize(_joints_number);
        p_avg_s.resize(_joints_number);
        temp_avg_s.resize(_joints_number);
        tau_avg_s.resize(_joints_number);
        _measured_friction_comp.resize(_joints_number);
        _error.resize(_joints_number);

        for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
        {
            if (_arm_status.joints[jnt_idx].current_error == 21)
                continue;
            p_avg[jnt_idx].resize(_avg_samples);
            temp_avg[jnt_idx].resize(_avg_samples);
            tau_avg[jnt_idx].resize(_avg_samples);
            _measured_friction_comp[jnt_idx].clear();
            vel_achi[jnt_idx] = false;
            cal_done[jnt_idx] = false;
            valid_vel_time[jnt_idx] = std::chrono::steady_clock::now();
            cycle_time[jnt_idx] = std::chrono::steady_clock::now();
            tq_acc[jnt_idx] = 0;
            tq_inc[jnt_idx] = 0;
            v_avg[jnt_idx] = 0;
            goal_v_avg_acc[jnt_idx] = 0;
            set_vel[jnt_idx] = 0.8 * direction * -1;
            prev_pos[jnt_idx] = _arm_status.joints[jnt_idx].position;
            p_avg_s[jnt_idx] = 0;
            temp_avg_s[jnt_idx] = 0;

            // RCLCPP_INFO_STREAM(_node->get_logger(), "Chart file: " << (std::string("/root/temp/t_charts/chart_") + std::string("joint") + std::to_string(jnt_idx) + std::string("_") + std::to_string(dir) + std::string(".txt")));

            // chart[jnt_idx] = std::make_shared<std::ofstream>(std::string("/home/avena/ros2_ws/src/avena_ros2_control/arm_controller/config/f_chart_.txt"));

            for (int i = 0; i < _avg_samples; i++)
            {
                p_avg[jnt_idx][i] = _arm_status.joints[jnt_idx].position;
                temp_avg[jnt_idx][i] = 0;
                tau_avg[jnt_idx][i] = 0;
            }

            //*chart[jnt_idx] << "position:\tvel:\ttemp:\ttorque_read:\ttorque_set:" << std::endl;
            // TODO: static friction pls
            // *chart[jnt_idx] << "0.00 0.0" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));

        while (rclcpp::ok())
        {
            // std::chrono::time_point<std::chrono::steady_clock> t_current = std::chrono::steady_clock::now();

            getArmStatus();
            auto t_current = std::chrono::steady_clock::now();
            if (_controller_state == 3)
            {
            }
            // if (rclcpp::spin_until_future_complete(_node, _get_result) == rclcpp::executor::FutureReturnCode::SUCCESS)

            for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
            {
                if (_arm_status.joints[jnt_idx].current_error == 21)
                    continue;

                temp_avg[jnt_idx][_loop_it % _avg_samples] = _arm_status.joints[jnt_idx].temperature;

                temp_avg_s[jnt_idx] = 0;

                for (size_t i = 0; i < _avg_samples; i++)
                {

                    temp_avg_s[jnt_idx] += temp_avg[jnt_idx][i];
                }

                temp_avg_s[jnt_idx] /= _avg_samples;
            }

            // calculate torques
            for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
            {
                if (_arm_status.joints[jnt_idx].current_error == 21)
                    continue;
                // RCLCPP_INFO_STREAM(_node->get_logger(), "set_vel: " << set_vel[jnt_idx]);
                // RCLCPP_INFO_STREAM(_node->get_logger(), "max_vel: " << max_vel);
                if (cal_done[jnt_idx])
                {
                    _arm_command.joints[jnt_idx].c_torque = 0;
                    // _arm_command.joints[jnt_idx].c_status = 3;
                    continue;
                }
                // dynamic PID reconfigure
                // updateParams(_pid_ctrl, jnt_idx);

                // calculate torques (PID+FF)

                _error[jnt_idx] = set_vel[jnt_idx] - _arm_status.joints[jnt_idx].velocity;
                // std::cout << "err: " << _error[jnt_idx] << "\nval: " << _pid_ctrl[jnt_idx].getComponents()[0] << " " << _pid_ctrl[jnt_idx].getComponents()[1] << ' ' << _pid_ctrl[jnt_idx].getComponents()[2] << std::endl;
                // _set_torque_val = _pid_ctrl[jnt_idx].getValue(_error);
                _set_torque_val = _pid_ctrl[jnt_idx].getValue(_error[jnt_idx]) + compensateFriction_coeffs(set_vel[jnt_idx], _arm_status.joints[jnt_idx].temperature, friction_coefficients[jnt_idx]);
                // _set_torque_val = _pid_ctrl[jnt_idx].getValue(_error[jnt_idx]) + compensateFriction(set_vel[jnt_idx],30.,jnt_idx);

                // TODO: params
                if ((std::chrono::steady_clock::now() - cycle_time[jnt_idx]) > std::chrono::seconds(60))
                {
                    tq_acc[jnt_idx] = 0;
                    goal_v_avg_acc[jnt_idx] = 0;
                    tq_inc[jnt_idx] = 0;
                    set_vel[jnt_idx] += vel_increment;
                    valid_vel_time[jnt_idx] = std::chrono::steady_clock::now();
                    cycle_time[jnt_idx] = std::chrono::steady_clock::now();
                }
                if (std::abs(_error[jnt_idx]) < (0.03 + std::abs(set_vel[jnt_idx]) * 0.012))
                {
                    if (!vel_achi[jnt_idx])
                        valid_vel_time[jnt_idx] = std::chrono::steady_clock::now();
                    else
                    {
                        if ((std::chrono::steady_clock::now() - valid_vel_time[jnt_idx]) > std::chrono::seconds(5))
                        {
                            // std::string temp_str = std::to_string(set_vel[jnt_idx]) + " " + std::to_string(tq_acc[jnt_idx] / double(tq_inc[jnt_idx])) + "\_avg_samples";
                            // std::cout << set_vel[jnt_idx] << " " << (tq_acc[jnt_idx] / double(tq_inc[jnt_idx])) << std::endl;
                            friction_comp temp_fc;
                            if (std::abs(set_vel[jnt_idx]) < 0.001)
                            {
                                temp_fc.tq = 0.;
                                temp_fc.vel = 0.;
                                temp_fc.temp = temp_avg_s[jnt_idx];
                                _measured_friction_comp[jnt_idx].push_back(temp_fc);
                            }
                            else
                            {
                                temp_fc.tq = tq_acc[jnt_idx] / double(tq_inc[jnt_idx]);
                                temp_fc.vel = goal_v_avg_acc[jnt_idx] / double(tq_inc[jnt_idx]);
                                temp_fc.temp = temp_avg_s[jnt_idx];
                                _measured_friction_comp[jnt_idx].push_back(temp_fc);
                            }
                            //*chart[jnt_idx]<< set_vel << " " << (tq_acc / double(tq_inc))<<std::endl;
                            // chart[jnt_idx]->write(temp_str.c_str(), temp_str.size());
                            // chart[jnt_idx]->flush();
                            tq_acc[jnt_idx] = 0;
                            goal_v_avg_acc[jnt_idx] = 0;
                            tq_inc[jnt_idx] = 0;
                            // next loop
                            if (std::abs(set_vel[jnt_idx]) > std::abs(max_vel))
                            {
                                cal_done[jnt_idx] = true;
                                RCLCPP_INFO(_node->get_logger(), "Joint %i done.", jnt_idx);
                            }
                            set_vel[jnt_idx] += vel_increment;
                            valid_vel_time[jnt_idx] = std::chrono::steady_clock::now();
                            cycle_time[jnt_idx] = std::chrono::steady_clock::now();
                        }
                    }
                    vel_achi[jnt_idx] = true;
                }
                else
                    vel_achi[jnt_idx] = false;

                // RCLCPP_INFO_STREAM(node_->get_logger(), "tau: "<<tau_[jnt_idx]);

                _torque_sign = ((_set_torque_val > 0) - (_set_torque_val < 0));
                // limit torque
                {
                    if (_set_torque_val * _torque_sign > 40)
                        _set_torque_val = 40 * _torque_sign;
                }

                // _set_torque_val = -14;
                // if (_jitter_present[jnt_idx]){
                //     _set_torque_val *= _jitter_multiplier[jnt_idx];
                //     RCLCPP_WARN(_node->get_logger(),"JITTER");
                //     std::cout<<_jitter_multiplier[jnt_idx]<<std::endl;
                // }

                _arm_command.joints[jnt_idx].c_torque = _set_torque_val;
                // _arm_command.joints[jnt_idx].c_torque = -15;
                // _arm_command.joints[jnt_idx].c_status = 3;
                // RCLCPP_INFO_STREAM(_node->get_logger(), "jnt: " << jnt_idx);
                // RCLCPP_INFO_STREAM(_node->get_logger(), "pos: " << _arm_status.joints[jnt_idx].position);

                // RCLCPP_INFO_STREAM(_node->get_logger(), "v_avg: " << v_avg[jnt_idx]);
                // RCLCPP_INFO_STREAM(_node->get_logger(), "error: " << _error);
                // RCLCPP_INFO_STREAM(_node->get_logger(), "temp: " << temp_avg_s[jnt_idx]);
                // RCLCPP_INFO_STREAM(_node->get_logger(), "tq: " << _arm_command.joints[jnt_idx].c_torque);

                if (vel_achi[jnt_idx] && (std::chrono::steady_clock::now() - valid_vel_time[jnt_idx] > std::chrono::seconds(1)))
                {
                    // *chart[jnt_idx] << p_avg_s[jnt_idx] << '\t' << v_avg[jnt_idx] << '\t' << temp_avg_s[jnt_idx] << '\t' << tau_avg_s[jnt_idx] << '\t' << set_vel << std::endl;
                    tq_acc[jnt_idx] += _set_torque_val;
                    tq_inc[jnt_idx]++;
                    goal_v_avg_acc[jnt_idx] += _arm_status.joints[jnt_idx].velocity;
                }
                // monitor data
                _set_joint_state_msg.position[jnt_idx] = temp_avg_s[jnt_idx];
                _set_joint_state_msg.velocity[jnt_idx] = set_vel[jnt_idx];
                // _set_joint_state_msg.velocity[jnt_idx] = _arm_status.joints[jnt_idx].velocity;
                _set_joint_state_msg.effort[jnt_idx] = _set_torque_val;
                _set_joint_state_msg.header.stamp = rclcpp::Clock().now();

                _arm_joint_state_msg.position[jnt_idx] = _arm_status.joints[jnt_idx].position;
                _arm_joint_state_msg.velocity[jnt_idx] = _arm_status.joints[jnt_idx].velocity;
                _arm_joint_state_msg.effort[jnt_idx] = _arm_status.joints[jnt_idx].torque;
                _arm_joint_state_msg.header.stamp = rclcpp::Clock().now();
                if (std::isnan(_set_torque_val))
                    return;
                if (std::isnan(_arm_command.joints[jnt_idx].c_torque))
                    return;
            }
            // debug
            //  RCLCPP_INFO(_node->get_logger(), "calc time: %i", std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t_current).count());
            _set_joint_states_pub->publish(_set_joint_state_msg);
            _arm_joint_states_pub->publish(_arm_joint_state_msg);

            _arm_command.timestamp = std::chrono::steady_clock::now();
            setArmCommand();

            // control loop frequency
            _remaining_time = std::floor(1000000 / _trajectory_rate - std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t_current).count());
            // time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t_current).count();
            // RCLCPP_INFO(_node->get_logger(), "time_taken: %i", time);
            _loop_it++;
            // RCLCPP_INFO(_node->get_logger(), "time_remaining: %i", _remaining_time);
            if (_remaining_time < 0)
                RCLCPP_ERROR(_node->get_logger(), "loop taking too long to execute");
            std::this_thread::sleep_for(std::chrono::microseconds(_remaining_time));

            bool all_done = true;
            for (size_t i = 0; i < _joints_number; i++)
            {
                // std::cout << cal_done[i] << std::endl;
                all_done *= cal_done[i];
            }
            if (all_done)
                break;
        }
        // std::cout << _measured_friction_comp.size() << std::endl;

        for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
        {
            if (_arm_status.joints[jnt_idx].current_error == 21)
                continue;
            _arm_command.joints[jnt_idx].c_torque = 0;
            // _arm_command.joints[jnt_idx].c_status = 2;

            if (_measured_friction_comp[jnt_idx].size() == 0)
                continue;
            std::string filename = std::string("/f_chart_joint_") + std::to_string(jnt_idx);
            int file_i = 0;
            while (std::filesystem::exists(std::filesystem::path(_config_path + std::string("/friction") + filename + std::string("_") + std::to_string(file_i) + std::string(".txt"))))
                file_i++;
            std::cout << _config_path + std::string("/friction") + filename + std::string("_") + std::to_string(file_i) + std::string(".txt") << std::endl;
            chart = std::make_shared<std::ofstream>(_config_path + std::string("/friction") + filename + std::string("_") + std::to_string(file_i) + std::string(".txt"));
            chart->precision(6);

            for (size_t j = 0; j < _measured_friction_comp[jnt_idx].size(); j++)
            {
                std::cout << _measured_friction_comp[jnt_idx].size() << std::endl;
                std::cout << "writing" << std::endl;
                *chart << std::to_string(_measured_friction_comp[jnt_idx][j].vel) << " " << std::to_string(_measured_friction_comp[jnt_idx][j].tq) << " " << std::to_string(_measured_friction_comp[jnt_idx][j].temp) << std::endl;
            }
            chart->flush();
            chart->close();
        }
    }
    setArmCommand();
    std::cout << "Calibration took: " << std::chrono::duration_cast<std::chrono::minutes>(std::chrono::steady_clock::now() - calib_time).count() << " minutes" << std::endl;
    RCLCPP_INFO(_node->get_logger(), "Done calibrating, shutting down");
    exit(0);
}

int main(int argc, char **argv)
{
    FrictionCalibration controller(argc, argv);

    controller.init();
    return 0;
}
