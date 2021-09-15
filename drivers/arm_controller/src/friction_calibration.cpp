#include "arm_controller/simple_controller.hpp"

//TODO: init essential functionalities, get ready to start
SimpleController::SimpleController(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    _node = rclcpp::Node::make_shared("simple_controller");
    _watchdog = std::make_shared<helpers::Watchdog>(_node.get(), this, "system_monitor");
    status = custom_interfaces::msg::Heartbeat::RUNNING;

    //COMMUNICATION INIT
    _command_service = _node->create_service<custom_interfaces::srv::ControlCommand>("/arm_controller/commands", std::bind(&SimpleController::setStateCb, this, std::placeholders::_1, std::placeholders::_2));
    _trajectory_sub = _node->create_subscription<trajectory_msgs::msg::JointTrajectory>("/trajectory", 1000, std::bind(&SimpleController::setTrajectoryCb, this, std::placeholders::_1));
    _time_factor_sub = _node->create_subscription<std_msgs::msg::Float64>("/arm_controller/time_factor", 10, std::bind(&SimpleController::setTimeFactorCb, this, std::placeholders::_1));
    _set_joint_states_pub = _node->create_publisher<sensor_msgs::msg::JointState>("set_joint_states", 10);
    _arm_joint_states_pub = _node->create_publisher<sensor_msgs::msg::JointState>("arm_joint_states", 10);
    _controller_state_pub = _node->create_publisher<std_msgs::msg::Int32>("/arm_controller/state", 10);
    _security_trigger_sub = _node->create_subscription<std_msgs::msg::Bool>(
        "/security_trigger",
        rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&SimpleController::securityTriggerStatusCb, this, std::placeholders::_1));

    _arm_interface = std::make_shared<ArmInterface>("0AA", (int)_trajectory_rate);
    std::this_thread::sleep_for(std::chrono::seconds(2));
}

SimpleController::~SimpleController()
{
}

void SimpleController::loadFrictionChart(std::string path)
{
    for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    {
        std::string temp_s;
        RCLCPP_INFO_STREAM(_node->get_logger(), "Loading friction chart: " << path + std::to_string(jnt_idx) + std::string(".txt"));
        std::ifstream fs(path + std::to_string(jnt_idx) + std::string(".txt"));
        std::vector<double> vel, temp;
        if (fs.good())
        {
            if (!fs.eof())
            {
                std::getline(fs, temp_s);
                size_t pos = 0;
                while ((pos = temp_s.find(' ')) != std::string::npos)
                {
                    vel.push_back(std::stod(temp_s.substr(0, pos)));
                    temp_s.erase(0, pos + 1);
                }
            }
            if (!fs.eof())
            {
                std::getline(fs, temp_s);
                size_t pos = 0;
                while ((pos = temp_s.find(' ')) != std::string::npos)
                {
                    temp.push_back(std::stod(temp_s.substr(0, pos)));
                    temp_s.erase(0, pos + 1);
                }
            }
            _friction_chart[jnt_idx].resize(vel.size());
            for (size_t v = 0; v < vel.size(); v++)
            {
                std::getline(fs, temp_s);
                size_t pos = 0;
                _friction_chart[jnt_idx][v].resize(temp.size());
                for (size_t t = 0; t < temp.size(); t++)
                {
                    if (!fs.eof())
                    {
                        pos = temp_s.find(' ');
                        _friction_chart[jnt_idx][v][t].vel = vel[v];
                        _friction_chart[jnt_idx][v][t].temp = temp[t];
                        _friction_chart[jnt_idx][v][t].tq = std::stod(temp_s.substr(0, pos));
                        temp_s.erase(0, pos + 1);
                    }
                }
            }
            fs.close();
        }
    }
}

double SimpleController::compensateFriction(double vel, double temp, int jnt_idx)
{
    if (vel == 0.0)
        return 0;
    if (_friction_chart[jnt_idx].size() == 0)
    {
        return 0;
    }
    int v = 0;
    while (v < _friction_chart[jnt_idx].size())
    {
        if (vel < _friction_chart[jnt_idx][v][0].vel)
            break;
        v++;
    }
    if (v >= _friction_chart[jnt_idx].size())
    {
        v--;
    }
    int t = 0;
    while (t < _friction_chart[jnt_idx].size())
    {
        if (temp < _friction_chart[jnt_idx][v][t].temp)
            break;
        t++;
    }
    if (t >= _friction_chart[jnt_idx][v].size())
    {
        t--;
    }
    bool up = true;
    int t_n = t;
    while (std::isnan(_friction_chart[jnt_idx][v][t_n].tq))
    {
        if (t_n >= _friction_chart[jnt_idx][v].size() - 1)
        {
            up = false;
            t_n = t;
        }
        if (up)
            t_n++;
        else
            t_n--;

        if (t_n < 0)
        {
            up = true;
            t_n = t;
            if (vel > 0)
                v--;
            else
                v++;
        }
    }
    t = t_n;

    return _friction_chart[jnt_idx][v][t].tq;
}

//only for debug purposes
void SimpleController::loadTrajTxt(std::string path)
{

    trajectory_msgs::msg::JointTrajectoryPoint temp_point;
    std::string temp_s;
    int it = 0;
    //position
    std::ifstream fs(path + "position.txt");
    if (fs.good())
    {
        while (!fs.eof())
        {
            for (size_t i = 0; i < _joints_number; i++)
            {
                std::getline(fs, temp_s);
                temp_point.positions.push_back(std::stod(temp_s));
                temp_point.time_from_start.sec = std::floor(it / _trajectory_rate);
                temp_point.time_from_start.nanosec = it / _trajectory_rate - temp_point.time_from_start.sec;
            }
            _saved_trajectory.points.push_back(temp_point);
            temp_point.positions.clear();
            it++;
        }
        fs.close();
    }
    //velocity
    it = 0;
    fs.open(path + "velocity.txt", std::fstream::in);
    if (fs.good())
    {
        while (!fs.eof())
        {
            for (size_t i = 0; i < _joints_number; i++)
            {
                std::getline(fs, temp_s);
                _saved_trajectory.points[it].velocities.push_back(std::stod(temp_s));
            }
            it++;
        }
        fs.close();
    }
    //acceleration
    it = 0;
    fs.open(path + "acceleration.txt", std::fstream::in);
    if (fs.good())
    {
        while (!fs.eof())
        {
            for (size_t i = 0; i < _joints_number; i++)
            {
                std::getline(fs, temp_s);
                _saved_trajectory.points[it].accelerations.push_back(std::stod(temp_s));
            }
            it++;
        }
        fs.close();
    }
}

//dynamic PID parameters
void SimpleController::updateParams(PID &pid, int joint_index)
{
    _node->get_parameter("Kp_gain_" + std::to_string(joint_index), _Kp[joint_index]);
    _node->get_parameter("Ki_gain_" + std::to_string(joint_index), _Ki[joint_index]);
    _node->get_parameter("Kd_gain_" + std::to_string(joint_index), _Kd[joint_index]);
    _node->get_parameter("FFv_gain_" + std::to_string(joint_index), _FFv[joint_index]);
    _node->get_parameter("FFa_gain_" + std::to_string(joint_index), _FFv[joint_index]);
    pid.update(_Kp[joint_index], _Ki[joint_index], _Kd[joint_index]);
}

void SimpleController::setTrajectoryCb(trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Received trajectory");
    if (msg->joint_names.size() != _joints_number)
    {
        RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Trajectory joint number does not match robot configuration");
        return;
    }
    _saved_trajectory = *msg;
}

//TODO:
void SimpleController::securityTriggerStatusCb(std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Received security pause trigger.");
    _controller_state = 1;
    return;
}

void SimpleController::setTimeFactorCb(std_msgs::msg::Float64::SharedPtr msg)
{
    if (_controller_state != 4)
    {
        _prev_time_factor = msg->data / 100;
    }
    else
    {
        _time_factor = msg->data / 100;
    }
    RCLCPP_INFO_STREAM(_node->get_logger(), "Time factor set to: " << _time_factor);
    return;
}

void SimpleController::setStateCb(const std::shared_ptr<custom_interfaces::srv::ControlCommand::Request> request,
                                  std::shared_ptr<custom_interfaces::srv::ControlCommand::Response> response)
{
    switch (request.get()->command)
    {
    //init
    case 0:
        RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Received start command.");
        _controller_state = 4;
        _t_start = std::chrono::steady_clock::now();
        _time_accumulator = std::chrono::microseconds(0);
        _time_factor = 1.;
        _prev_time_factor = _time_factor;
        _t_current = std::chrono::steady_clock::now();
        response->feedback = "Received start command.";
        break;

    //stop
    case 1:
        RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Received stop command.");
        _controller_state = 1;
        if (_time_factor != 0.)
            _prev_time_factor = _time_factor;
        _time_factor = 0.;
        response->feedback = "Received stop command.";
        break;
    //resume
    case 2:
        RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Received resume command.");
        if (_controller_state == 3)
        {
            _controller_state = 2;
            _t_stop = std::chrono::steady_clock::now();
            response->feedback = "Received resume command.";
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Controller needs to be paused before resuming.");
            response->feedback = "Controller needs to be paused before resuming.";
        }
        break;
    //pause
    case 3:
        if (_controller_state == 4)
        {
            RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Received pause command.");
            _controller_state = 3;
            _t_stop = std::chrono::steady_clock::now();
            if (_time_factor != 0.)
                _prev_time_factor = _time_factor;
            response->feedback = "Received pause command.";
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Controller needs to be running before pausing.");
            response->feedback = "Controller needs to be running before pausing.";
        }
        break;
    //execute
    case 4:
        RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Received execute command.");
        if (_saved_trajectory.points.size() == 0)
        {
            response->feedback = "Please send trajectory before trying to execute it.";
            break;
        }
        _trajectory = _saved_trajectory;
        _controller_state = 4;
        _time_factor = _prev_time_factor;
        _t_start = std::chrono::steady_clock::now();
        _time_accumulator = std::chrono::microseconds(0);
        _t_current = std::chrono::steady_clock::now();
        response->feedback = "Received execute command.";
        break;
    default:
        RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Received unknown command.");
        response->feedback = "Received unknown command.";
        break;
    }
}

//initialize movement functionalities, start controller
void SimpleController::init()
{

    //JOINT COMMUNICATION INIT
    RCLCPP_INFO(_node->get_logger(), "Starting executor");
    _exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    _exec->add_node(_node);
    RCLCPP_INFO(_node->get_logger(), "Getting arm state from CANDRIVER...");
    _arm_status = _arm_interface->getArmState();

    RCLCPP_INFO(_node->get_logger(), "Got arm state from CANDRIVER");
    _joints_number = _arm_status.joints.size();
    for (size_t i = 0; i < _joints_number; i++)
    {
        if (_arm_status.joints[i].status != 0)
        {
            RCLCPP_INFO_STREAM(_node->get_logger(), "Joint error " << _arm_status.joints[i].fault << " on joint " << i);
        }
    }

    RCLCPP_INFO(_node->get_logger(), "Joint number: %i", _joints_number);

    _arm_command.joints.resize(_joints_number);
    _friction_chart.resize(_joints_number);
    _Kp.resize(_joints_number);
    _Ki.resize(_joints_number);
    _Kd.resize(_joints_number);
    _FFv.resize(_joints_number);
    _FFa.resize(_joints_number);
    _i_clamp_h.resize(_joints_number);
    _i_clamp_l.resize(_joints_number);
    _c_friction_val.resize(_joints_number);

    _set_joint_state_msg.name.resize(_joints_number);
    _set_joint_state_msg.position.resize(_joints_number);
    _set_joint_state_msg.velocity.resize(_joints_number);
    _set_joint_state_msg.effort.resize(_joints_number);

    _arm_joint_state_msg.name.resize(_joints_number);
    _arm_joint_state_msg.position.resize(_joints_number);
    _arm_joint_state_msg.velocity.resize(_joints_number);
    _arm_joint_state_msg.effort.resize(_joints_number);

    _q.resize(_joints_number);
    _qd.resize(_joints_number);
    _qdd.resize(_joints_number);

    _avg_acc.resize(_joints_number);
    _avg_pos.resize(_joints_number);
    _avg_vel.resize(_joints_number);
    _avg_temp.resize(_joints_number);
    _avg_tau.resize(_joints_number);
    _prev_pos.resize(_joints_number);

    _avg_acc_b.resize(_joints_number);
    _avg_pos_b.resize(_joints_number);
    _avg_vel_b.resize(_joints_number);
    _avg_temp_b.resize(_joints_number);
    _avg_tau_b.resize(_joints_number);
    _frick_acu.resize(_joints_number);

    //PARAMETERS INIT
    _node->declare_parameter<double>("error_margin", 0);
    _node->declare_parameter<std::string>("config_path", "");
    _node->get_parameter("config_path", _config_path);
    _node->get_parameter("error_margin", _error_margin);
    for (size_t i = 0; i < _joints_number; i++)
    {
        //set defaults in case config file is not provided
        _node->declare_parameter<double>("Kp_gain_" + std::to_string(i), 38);
        _node->declare_parameter<double>("Ki_gain_" + std::to_string(i), 1);
        _node->declare_parameter<double>("Kd_gain_" + std::to_string(i), 25);
        _node->declare_parameter<double>("FFv_gain_" + std::to_string(i), 17);
        _node->declare_parameter<double>("FFa_gain_" + std::to_string(i), 1);
        _node->declare_parameter<double>("Coulomb_friction_" + std::to_string(i), 9);
        _node->declare_parameter<double>("i_clamp_h_" + std::to_string(i), 20);
        _node->declare_parameter<double>("i_clamp_l_" + std::to_string(i), -20);

        //get parameter values from config file
        _node->get_parameter("Kp_gain_" + std::to_string(i), _Kp[i]);
        _node->get_parameter("Ki_gain_" + std::to_string(i), _Ki[i]);
        _node->get_parameter("Kd_gain_" + std::to_string(i), _Kd[i]);
        _node->get_parameter("FFv_gain_" + std::to_string(i), _FFv[i]);
        _node->get_parameter("FFa_gain_" + std::to_string(i), _FFv[i]);
        _node->get_parameter("Coulomb_friction_" + std::to_string(i), _c_friction_val[i]);
        _node->get_parameter("i_clamp_h_" + std::to_string(i), _i_clamp_h[i]);
        _node->get_parameter("i_clamp_l_" + std::to_string(i), _i_clamp_l[i]);

        //initialize PIDs
        _pid_ctrl.push_back(PID(_Kp[i], _Ki[i], _Kd[i], 1.0 / _trajectory_rate, _i_clamp_l[i], _i_clamp_h[i], _avg_samples));
    }
    RCLCPP_INFO(_node->get_logger(), "Done setting PIDs");

    //FRICTION INIT
    loadFrictionChart(_config_path + std::string("/friction_chart_"));
    RCLCPP_INFO(_node->get_logger(), "Loaded friction chart");

    //MEASUREMENT INIT
    for (size_t i = 0; i < _joints_number; i++)
    {
        _frick_acu[i] = 0;
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

    //GET STARTING POSITION - HOLD TRAJECTORY
    _arm_status = _arm_interface->getArmState();
    _trajectory.points.resize(1);
    _trajectory.points[0].positions.resize(_joints_number);
    _trajectory.points[0].velocities.resize(_joints_number);
    _trajectory.points[0].accelerations.resize(_joints_number);
    for (size_t i = 0; i < _joints_number; i++)
    {
        _trajectory.points[0].positions[i] = _arm_status.joints[i].position;
        _trajectory.points[0].velocities[i] = 0.;
        _trajectory.points[0].accelerations[i] = 0.;
        _trajectory.points[0].time_from_start.sec = 0.;
        _trajectory.points[0].time_from_start.nanosec = 0.;
        std::cout << _arm_status.joints[i].position << '\t' << std::endl;
    }

    //TIME INIT
    int loop_it = 0;
    _t_start = std::chrono::steady_clock::now();
    _time_accumulator = std::chrono::microseconds(0);
    _controller_state = 4;
    _time_factor = 1.;
    _prev_time_factor = 0.;
    _t_current = std::chrono::steady_clock::now();
    _t_stop = std::chrono::steady_clock::now();
    _slowdown_duration = std::chrono::microseconds(1000000);

    // //MEASUREMENT INIT
    // for (size_t i = 0; i < _joints_number; i++)
    // {
    //     _frick_acu[i] = 0;
    //     _avg_acc_b[i].resize(_avg_samples);
    //     _avg_pos_b[i].resize(_avg_samples);
    //     _avg_vel_b[i].resize(_avg_samples);
    //     _avg_temp_b[i].resize(_avg_samples);
    //     _avg_tau_b[i].resize(_avg_samples);
    //     for (size_t j = 0; j < _avg_samples; j++)
    //     {
    //         _avg_acc_b[i][j] = 0;
    //         _avg_pos_b[i][j] = 0;
    //         _avg_vel_b[i][j] = 0;
    //         _avg_temp_b[i][j] = 0;
    //         _avg_tau_b[i][j] = 0;
    //     }
    // }
    
    for (int asdf = 0; asdf < 6; asdf++)
    {

        std::vector<double> set_vel;
        double vel_increment = -0.05, max_vel = -0.8;
        const int n = 100;
        std::vector<double> v_avg, prev_pos;
        std::vector<std::array<double, n>> p_avg, temp_avg, tau_avg;
        std::vector<double> p_avg_s, temp_avg_s, tau_avg_s, goal_v_avg_acc;
        std::vector<std::shared_ptr<std::ofstream>> chart;
        friction_comp friction_point;
        std::vector<double> tq_acc;
        std::vector<int> tq_inc;
        std::vector<bool> vel_achi;
        std::vector<bool> cal_done;
        std::vector<std::chrono::steady_clock::time_point> valid_vel_time;

        goal_v_avg_acc.resize(_joints_number);
        vel_achi.resize(_joints_number);
        cal_done.resize(_joints_number);
        valid_vel_time.resize(_joints_number);
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
        chart.resize(_joints_number);
        _measured_friction_comp.resize(_joints_number);

        for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
        {

            _measured_friction_comp[jnt_idx].clear();
            int file_i = 0;
            std::string filename = std::string("/f_chart_joint_") + std::to_string(jnt_idx);
            vel_achi[jnt_idx] = false;
            cal_done[jnt_idx] = false;
            valid_vel_time[jnt_idx] = std::chrono::steady_clock::now();
            tq_acc[jnt_idx] = 0;
            tq_inc[jnt_idx] = 0;
            goal_v_avg_acc[jnt_idx] = 0;
            set_vel[jnt_idx] = 0.8;
            prev_pos[jnt_idx] = _arm_status.joints[jnt_idx].position;
            while (std::filesystem::exists(std::filesystem::path(_config_path + filename + std::string("_") + std::to_string(file_i) + std::string(".txt"))))
                file_i++;
            std::cout << _config_path + filename + std::string("_") + std::to_string(file_i) + std::string(".txt") << std::endl;
            //RCLCPP_INFO_STREAM(_node->get_logger(), "Chart file: " << (std::string("/root/temp/t_charts/chart_") + std::string("joint") + std::to_string(jnt_idx) + std::string("_") + std::to_string(dir) + std::string(".txt")));
            chart[jnt_idx] = std::make_shared<std::ofstream>(_config_path + filename + std::string("_") + std::to_string(file_i) + std::string(".txt"));
            // chart[jnt_idx] = std::make_shared<std::ofstream>(std::string("/home/avena/ros2_ws/src/avena_ros2_control/arm_controller/config/f_chart_.txt"));


            for (int i = 0; i < n; i++)
            {
                p_avg[jnt_idx][i] = _arm_status.joints[i].position;
                temp_avg[jnt_idx][i] = 0;
                tau_avg[jnt_idx][i] = 0;
            }
            chart[jnt_idx]->precision(6);
            //*chart[jnt_idx] << "position:\tvel:\ttemp:\ttorque_read:\ttorque_set:" << std::endl;
            //TODO: static friction pls
            // *chart[jnt_idx] << "0.00 0.0" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));

        while (rclcpp::ok())
        {

            //std::chrono::time_point<std::chrono::steady_clock> t_current = std::chrono::steady_clock::now();

            _arm_status = _arm_interface->getArmState();
            auto t_current = std::chrono::steady_clock::now();
            if (_controller_state == 3)
            {
            }
            // if (rclcpp::spin_until_future_complete(_node, _get_result) == rclcpp::executor::FutureReturnCode::SUCCESS)

            for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
            {
                //crimge
                if (std::abs(_arm_status.joints[jnt_idx].position - prev_pos[jnt_idx]) > 10)
                {
                    for (size_t i = 0; i < n; i++)
                    {
                        p_avg[jnt_idx][i] = _arm_status.joints[jnt_idx].position;
                    }
                    prev_pos[jnt_idx] = _arm_status.joints[jnt_idx].position;
                }
                p_avg[jnt_idx][loop_it % n] = _arm_status.joints[jnt_idx].position;

                temp_avg[jnt_idx][loop_it % n] = _arm_status.joints[jnt_idx].temperature;
                tau_avg[jnt_idx][loop_it % n] = _arm_status.joints[jnt_idx].torque;

                p_avg_s[jnt_idx] = 0;
                temp_avg_s[jnt_idx] = 0;
                tau_avg_s[jnt_idx] = 0;
                for (size_t i = 0; i < n; i++)
                {
                    p_avg_s[jnt_idx] += p_avg[jnt_idx][i];
                    temp_avg_s[jnt_idx] += temp_avg[jnt_idx][i];
                    tau_avg_s[jnt_idx] += tau_avg[jnt_idx][i];
                }
                p_avg_s[jnt_idx] /= n;
                temp_avg_s[jnt_idx] /= n;
                tau_avg_s[jnt_idx] /= n;

                v_avg[jnt_idx] = ((p_avg_s[jnt_idx] - prev_pos[jnt_idx]) * _trajectory_rate);
                prev_pos[jnt_idx] = p_avg_s[jnt_idx];
            }

            //calculate torques
            for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
            {
                if (cal_done[jnt_idx])
                {
                    _arm_command.joints[jnt_idx].c_torque = 0;
                    _arm_command.joints[jnt_idx].c_status = 0;
                    continue;
                }
                //dynamic PID reconfigure
                updateParams(_pid_ctrl[jnt_idx], jnt_idx);

                //calculate torques (PID+FF)

                _error = set_vel[jnt_idx] - v_avg[jnt_idx];
                _set_torque_val = _pid_ctrl[jnt_idx].getValue(_error) + compensateFriction(set_vel[jnt_idx], _arm_status.joints[jnt_idx].temperature, jnt_idx);
                //TODO: params
                if (std::abs(_error) < (0.03 + std::abs(set_vel[jnt_idx]) * 0.015))
                {
                    if (!vel_achi[jnt_idx])
                        valid_vel_time[jnt_idx] = std::chrono::steady_clock::now();
                    else
                    {
                        if ((std::chrono::steady_clock::now() - valid_vel_time[jnt_idx]) > std::chrono::seconds(5))
                        {
                            // std::string temp_str = std::to_string(set_vel[jnt_idx]) + " " + std::to_string(tq_acc[jnt_idx] / double(tq_inc[jnt_idx])) + "\n";
                            // std::cout << set_vel[jnt_idx] << " " << (tq_acc[jnt_idx] / double(tq_inc[jnt_idx])) << std::endl;
                            friction_comp temp_fc;
                            if (std::abs(set_vel[jnt_idx]) < 0.001)
                            {
                                temp_fc.tq = 0.;
                                temp_fc.vel = 0.;
                                temp_fc.temp = temp_avg_s[jnt_idx];
                                _measured_friction_comp[jnt_idx].push_back(temp_fc);
                            }
                            temp_fc.tq = tq_acc[jnt_idx] / double(tq_inc[jnt_idx]);
                            temp_fc.vel = goal_v_avg_acc[jnt_idx] / double(tq_inc[jnt_idx]);
                            temp_fc.temp = temp_avg_s[jnt_idx];
                            _measured_friction_comp[jnt_idx].push_back(temp_fc);
                            //*chart[jnt_idx]<< set_vel << " " << (tq_acc / double(tq_inc))<<std::endl;
                            // chart[jnt_idx]->write(temp_str.c_str(), temp_str.size());
                            //chart[jnt_idx]->flush();
                            tq_acc[jnt_idx] = 0;
                            goal_v_avg_acc[jnt_idx] = 0;
                            tq_inc[jnt_idx] = 0;
                            if (set_vel[jnt_idx] <= max_vel)
                                cal_done[jnt_idx] = true;
                            set_vel[jnt_idx] += vel_increment;
                        }
                    }
                    vel_achi[jnt_idx] = true;
                }
                else
                    vel_achi[jnt_idx] = false;

                //RCLCPP_INFO_STREAM(node_->get_logger(), "tau: "<<tau_[jnt_idx]);

                _torque_sign = ((_set_torque_val > 0) - (_set_torque_val < 0));
                //limit torque
                {
                    if (_set_torque_val * _torque_sign > 33)
                        _set_torque_val = 33 * _torque_sign;
                }

                _arm_command.joints[jnt_idx].c_torque = _set_torque_val;
                // set_request->torques.at(jnt_idx) = -7;
                _arm_command.joints[jnt_idx].c_status = 1;
                RCLCPP_INFO_STREAM(_node->get_logger(), "jnt: " << jnt_idx);
                RCLCPP_INFO_STREAM(_node->get_logger(), "pos: " << _arm_status.joints[jnt_idx].position);
                RCLCPP_INFO_STREAM(_node->get_logger(), "set_vel: " << set_vel[jnt_idx]);
                RCLCPP_INFO_STREAM(_node->get_logger(), "v_avg: " << v_avg[jnt_idx]);
                RCLCPP_INFO_STREAM(_node->get_logger(), "temp: " << temp_avg_s[jnt_idx]);
                RCLCPP_INFO_STREAM(_node->get_logger(), "tq: " << _arm_command.joints[jnt_idx].c_torque);

                if (vel_achi[jnt_idx] && (std::chrono::steady_clock::now() - valid_vel_time[jnt_idx] > std::chrono::seconds(1)))
                {
                    // *chart[jnt_idx] << p_avg_s[jnt_idx] << '\t' << v_avg[jnt_idx] << '\t' << temp_avg_s[jnt_idx] << '\t' << tau_avg_s[jnt_idx] << '\t' << set_vel << std::endl;
                    tq_acc[jnt_idx] += _set_torque_val;
                    tq_inc[jnt_idx]++;
                    goal_v_avg_acc[jnt_idx] += v_avg[jnt_idx];
                }
                //monitor data
                _set_joint_state_msg.position[jnt_idx] = 0;
                _set_joint_state_msg.velocity[jnt_idx] = set_vel[jnt_idx];
                _set_joint_state_msg.effort[jnt_idx] = _set_torque_val;
                _set_joint_state_msg.header.stamp = rclcpp::Clock().now();

                _arm_joint_state_msg.position[jnt_idx] = _arm_status.joints[jnt_idx].position;
                _arm_joint_state_msg.velocity[jnt_idx] = v_avg[jnt_idx];
                _arm_joint_state_msg.effort[jnt_idx] = _arm_status.joints[jnt_idx].torque;
                _arm_joint_state_msg.header.stamp = rclcpp::Clock().now();
            }
            //debug
            RCLCPP_INFO(_node->get_logger(), "calc time: %i", std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t_current).count());
            _set_joint_states_pub->publish(_set_joint_state_msg);
            _arm_joint_states_pub->publish(_arm_joint_state_msg);

            _arm_command.timestamp = std::chrono::steady_clock::now();
            _arm_interface->setArmCommand(_arm_command);

            //control loop frequency
            _remaining_time = std::floor(1000000 / _trajectory_rate - std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t_current).count());
            // time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t_current).count();
            // RCLCPP_INFO(_node->get_logger(), "time_taken: %i", time);
            loop_it++;
            RCLCPP_INFO(_node->get_logger(), "time_remaining: %i", _remaining_time);
            if (_remaining_time < 0)
                RCLCPP_ERROR(_node->get_logger(), "loop taking too long to execute");
            std::this_thread::sleep_for(std::chrono::microseconds(_remaining_time));

            bool all_done = true;
            for (size_t i = 0; i < _joints_number; i++)
            {
                std::cout << cal_done[i] << std::endl;
                all_done *= cal_done[i];
            }
            if (all_done)
                break;
        }
        std::cout << _measured_friction_comp.size() << std::endl;

        for (size_t i = 0; i < _measured_friction_comp.size(); i++)
        {
            _arm_command.joints[i].c_torque = 0;
            _arm_command.joints[i].c_status = 0;
            for (size_t j = 0; j < _measured_friction_comp[i].size(); j++)
            {
                std::cout << _measured_friction_comp[i].size() << std::endl;
                std::cout << "writing" << std::endl;
                *chart[i] << std::to_string(_measured_friction_comp[i][j].vel) << " " << std::to_string(_measured_friction_comp[i][j].tq) << " " << std::to_string(_measured_friction_comp[i][j].temp) << std::endl;
            }
            chart[i]->flush();
            chart[i]->close();
        }
    }
    _arm_interface->setArmCommand(_arm_command);

    RCLCPP_INFO(_node->get_logger(), "Done calibrating, shutting down");

    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    SimpleController controller(argc, argv);

    controller.init();
    return 0;
}
