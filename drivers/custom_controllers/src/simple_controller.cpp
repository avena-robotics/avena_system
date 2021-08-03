#include "custom_controllers/simple_controller.hpp"

using namespace std::chrono_literals;

//TODO: init essential functionalities, get ready to start
SimpleController::SimpleController(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    _node = rclcpp::Node::make_shared("simple_controller");
    _watchdog = std::make_shared<helpers::Watchdog>(_node.get(), this, "system_monitor");
    status = custom_interfaces::msg::Heartbeat::STOPPED;
    _command_service = _node->create_service<custom_interfaces::srv::ControlCommand>("/arm_controller/commands", std::bind(&SimpleController::setState, this, std::placeholders::_1, std::placeholders::_2));
}

SimpleController::~SimpleController()
{
}

void SimpleController::loadFrictionChart(std::string path)
{
    for (int jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    {
        std::string temp_s;
        friction_comp temp_fc;
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
                    std::cout << vel.back() << std::endl;
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
                    std::cout << temp.back() << std::endl;
                }
            }
            _friction_chart[jnt_idx].resize(vel.size());
            for (int v = 0; v < vel.size(); v++)
            {
                std::getline(fs, temp_s);
                size_t pos = 0;
                _friction_chart[jnt_idx][v].resize(temp.size());
                for (int t = 0; t < temp.size(); t++)
                {
                    if (!fs.eof())
                    {
                        pos = temp_s.find(' ');
                        _friction_chart[jnt_idx][v][t].vel = vel[v];
                        _friction_chart[jnt_idx][v][t].temp = temp[t];
                        _friction_chart[jnt_idx][v][t].tq = std::stod(temp_s.substr(0, pos));
                        temp_s.erase(0, pos + 1);
                        // std::cout << std::to_string(_friction_chart[jnt_idx][v][t].vel) << " " << std::to_string(_friction_chart[jnt_idx][v][t].tq) << " " << std::to_string(_friction_chart[jnt_idx][v][t].temp) << std::endl;
                    }
                }
            }
            fs.close();
            // for (size_t i = 0; i < _friction_chart[jnt_idx].size(); i++)
            // {
            //     std::cout << _friction_chart[jnt_idx][i].vel << '\t' << _friction_chart[jnt_idx][i].tq << std::endl;
            // }
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
        // std::cout << _friction_chart[jnt_idx][v][t_n].vel << std::string("\t") << _friction_chart[jnt_idx][v][t_n].temp << std::string("\t") << _friction_chart[jnt_idx][v][t_n].tq << std::endl;
    }
    t = t_n;
    // std::cout << v << " " << t << std::endl;
    std::cout << _friction_chart[jnt_idx][v][t].vel << std::string("\t") << _friction_chart[jnt_idx][v][t].temp << std::string("\t") << _friction_chart[jnt_idx][v][t].tq << std::endl;


    if (std::abs(_avg_vel[jnt_idx]) > 0.01 || _acc_sign!=_vel_sign){
        _frick_acc[jnt_idx]=0;
        
    }
    else
    {

        _frick_acc[jnt_idx]+=(_vel_sign*0.5);
    }
    return _friction_chart[jnt_idx][v][t].tq + _frick_acc[jnt_idx];
    // //linear interpolation
    // double v_diff = _friction_chart[jnt_idx][i].vel - _friction_chart[jnt_idx][i - 1].vel;
    // double tq_diff = _friction_chart[jnt_idx][i].tq - _friction_chart[jnt_idx][i - 1].tq;
    // return _friction_chart[jnt_idx][i - 1].tq + ((tq_diff / v_diff) * (vel - _friction_chart[jnt_idx][i - 1].vel));
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
            for (int i = 0; i < _joints_number; i++)
            {
                std::getline(fs, temp_s);
                temp_point.positions.push_back(std::stod(temp_s));
                temp_point.time_from_start.sec = std::floor(it / _trajectory_rate);
                temp_point.time_from_start.nanosec = it / _trajectory_rate - temp_point.time_from_start.sec;
            }
            _trajectory.points.push_back(temp_point);
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
            for (int i = 0; i < _joints_number; i++)
            {
                std::getline(fs, temp_s);
                _trajectory.points[it].velocities.push_back(std::stod(temp_s));
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
            for (int i = 0; i < _joints_number; i++)
            {
                std::getline(fs, temp_s);
                _trajectory.points[it].accelerations.push_back(std::stod(temp_s));
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

//TODO: replace with action?
void SimpleController::setTrajectory(const std::shared_ptr<custom_interfaces::srv::SetTrajectory::Request> request,
                                     std::shared_ptr<custom_interfaces::srv::SetTrajectory::Response> response)
{
    _trajectory = request.get()->trajectory;

    if (_trajectory.joint_names.size() != _joints_number)
    {
        RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Trajectory joint number does not match robot configuration");
        response->error = "Trajectory joint number does not match robot configuration";
        return;
    }
}

//TODO:
void SimpleController::setState(const std::shared_ptr<custom_interfaces::srv::ControlCommand::Request> request,
                                std::shared_ptr<custom_interfaces::srv::ControlCommand::Response> response)
{
    switch (request.get()->command)
    {
    //init
    case 0:
        RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Received initialization command.");
        // _controller_state = 0; //stops control loop
        //engage brakes
        response->feedback = "Received initialization command.";
        break;

    //stop
    case 1:
        RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Received stop command.");
        _controller_state = 1;
        // _t_stop = std::chrono::steady_clock::now();
        //starts init and control loop with 0 vel,acc - holds position
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
        //(smoothly?) resumes previous trajectory

        break;
    //pause
    case 3:
        if (_controller_state == 4)
        {
            RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Received pause command.");
            _controller_state = 3;
            _t_stop = std::chrono::steady_clock::now();
            _prev_time_factor = _time_factor;
            response->feedback = "Received pause command.";
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Controller needs to be running before pausing.");
            response->feedback = "Controller needs to be running before pausing.";
        }
        //(smoothly?) pauses current trajectory
        //saves current trajectory progress
        //executes hold position trajectory
        break;
    //execute
    case 4:
        RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Received execute command.");
        _controller_state = 4;
        //starts executing current trajectory
        response->feedback = "Received execute command.";
        break;
    default:
        RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Received unknown command.");
        response->feedback = "Received unknown command.";
        break;
    }

    // if (std::size(trajectory_.joint_names) != _joints_number)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Trajectory joint number does not match robot configuration");
    //     response->error = "Trajectory joint number does not match robot configuration";
    //     return;
    // }
}

//initialize movement functionalities, start controller
void SimpleController::init()
{

    //initialize driver client
    _set_client = _node->create_client<custom_interfaces::srv::SetArmTorques>("/arm_controller/set_torques");
    _get_client = _node->create_client<custom_interfaces::srv::GetArmState>("/arm_controller/get_current_arm_state");

    auto set_request = std::make_shared<custom_interfaces::srv::SetArmTorques::Request>();
    auto get_request = std::make_shared<custom_interfaces::srv::GetArmState::Request>();

    while (!_set_client->wait_for_service(1s) && !_get_client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            return;
        }
        RCLCPP_INFO(_node->get_logger(), "service not available, waiting again...");
    }
    RCLCPP_INFO(_node->get_logger(), "Getting joints...");
    //get joints
    //temp solution for candriver - need to set before getting
    // set_request->torques.resize(1);
    // set_request->turn_motor.resize(1);
    // set_request->torques.at(0) = 0;
    // set_request->turn_motor.at(0) = 0;
    // _set_result=_set_client->async_send_request(set_request);
    // rclcpp::spin_until_future_complete(_node, _set_result);
    _get_result = _get_client->async_send_request(get_request);
    // _set_result = _set_client->async_send_request(set_request);
    RCLCPP_INFO(_node->get_logger(), "Got arm state from CANDRIVER");

    if (rclcpp::spin_until_future_complete(_node, _get_result) == rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        _joints_number = _get_result.get()->arm_current_positions.size();
        for (size_t i = 0; i < _joints_number; i++)
        {
            if (_get_result.get()->arm_current_status[i] != 0)
            {
                RCLCPP_INFO_STREAM(_node->get_logger(), "Joint error " << _get_result.get()->arm_current_status[i] << " on joint " << i);
            }
        }
    }
    else
    {
        RCLCPP_ERROR(_node->get_logger(), "Failed to call service get_arm_state"); // CHANGE
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO(_node->get_logger(), "Joint number: %i", _joints_number);
    set_request->torques.resize(_joints_number);
    set_request->turn_motor.resize(_joints_number);

    _friction_chart.resize(_joints_number);
    _Kp.resize(_joints_number);
    _Ki.resize(_joints_number);
    _Kd.resize(_joints_number);
    _FFv.resize(_joints_number);
    _FFa.resize(_joints_number);
    _i_clamp_h.resize(_joints_number);
    _i_clamp_l.resize(_joints_number);
    _c_friction_val.resize(_joints_number);

    std::string urdf_path = "/home/avena/avena_system/src/avena_system/core/avena_bringup/urdf/v4.urdf.xacro";

    //initialize ID
    RCLCPP_INFO_STREAM(_node->get_logger(), "Loading robot model from: " << urdf_path);
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model, true);

    pinocchio::Data data(model);

    _q.resize(_joints_number);
    _qd.resize(_joints_number);
    _qdd.resize(_joints_number);

    RCLCPP_INFO(_node->get_logger(), "Done loading robot model.");

    //parameters
    _node->declare_parameter<double>("error_margin", 0);
    _node->declare_parameter<std::string>("config_path", "");
    _node->get_parameter("config_path", _config_path);
    _node->get_parameter("error_margin", _error_margin);
    for (int i = 0; i < _joints_number; i++)
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
        _pid_ctrl.push_back(PID(_Kp[i], _Ki[i], _Kd[i], 1.0 / _trajectory_rate, _i_clamp_l[i], _i_clamp_h[i]));
    }
    RCLCPP_INFO(_node->get_logger(), "Done setting PIDs");
    //initialize log topic
    _log_msg.position.resize(_joints_number);
    _log_msg.velocity.resize(_joints_number);
    _log_msg.effort.resize(_joints_number);
    _log_publisher = _node->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10);

    //load trajectory from txt (for testing purposes)
    _controller_state = 4;
    loadTrajTxt("/home/avena/avena_system/src/avena_system/drivers/custom_controllers/trajectory/");
    RCLCPP_INFO(_node->get_logger(), "Loaded test trajectory");
    loadFrictionChart(_config_path + std::string("/friction_chart_"));
    RCLCPP_INFO(_node->get_logger(), "Loaded friction chart");
    // for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    // {

    //     set_request->torques.at(jnt_idx) = 0;
    //     set_request->turn_motor.at(jnt_idx) = 1;
    // }

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
    _frick_acc.resize(_joints_number);

    int loop_it = 0;


    for (int i = 0; i < _joints_number; i++)
    {
        _frick_acc[i]=0;
        _avg_acc_b[i].resize(_avg_samples);
        _avg_pos_b[i].resize(_avg_samples);
        _avg_vel_b[i].resize(_avg_samples);
        _avg_temp_b[i].resize(_avg_samples);
        _avg_tau_b[i].resize(_avg_samples);
        for (int j = 0; j < _avg_samples; j++)
        {
            _avg_acc_b[i][j] = 0;
            _avg_pos_b[i][j] = 0;
            _avg_vel_b[i][j] = 0;
            _avg_temp_b[i][j] = 0;
            _avg_tau_b[i][j] = 0;
        }
    }

    while (!_set_client->wait_for_service(1s) && !_get_client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            return;
        }
        RCLCPP_INFO(_node->get_logger(), "service not available, waiting again...");
    }

    _t_start = std::chrono::steady_clock::now();
    _time_accumulator = std::chrono::microseconds(0);
    _time_factor = 1.;
    _prev_time_factor = _time_factor;
    _t_current = std::chrono::steady_clock::now();
    _slowdown_duration = std::chrono::microseconds(1000000);

    //loop
    std::this_thread::sleep_for(std::chrono::microseconds(10));
    RCLCPP_INFO(_node->get_logger(), "Done initializing, entering control loop");

    while (rclcpp::ok())
    {
        // RCLCPP_INFO(_node->get_logger(), "Controller state: %i", _controller_state);
        _time_accumulator += std::chrono::duration_cast<std::chrono::microseconds>((std::chrono::steady_clock::now() - _t_current) * _time_factor);
        _t_current = std::chrono::steady_clock::now();
        //std::chrono::time_point<std::chrono::steady_clock> _t_current = std::chrono::steady_clock::now();
        _t_measure = std::chrono::steady_clock::now();
        _get_result = _get_client->async_send_request(get_request);
        if (rclcpp::spin_until_future_complete(_node, _get_result) == rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
            {

                _avg_pos_b[jnt_idx][loop_it % _avg_samples] = _get_result.get()->arm_current_positions.at(jnt_idx);

                _avg_temp_b[jnt_idx][loop_it % _avg_samples] = _get_result.get()->arm_current_temperature.at(jnt_idx);
                _avg_tau_b[jnt_idx][loop_it % _avg_samples] = _get_result.get()->arm_current_torques.at(jnt_idx);

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
            }
        }
        else
        {
            RCLCPP_ERROR(_node->get_logger(), "Failed to call service get_arm_state"); // CHANGE
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(_node->get_logger(), "get_result1: %i", std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - _t_measure).count());
        //STATES
        if (_controller_state == 3)
        {
            _time_factor = double(std::max(0., double(std::chrono::duration_cast<std::chrono::microseconds>(_t_stop - _t_current + _slowdown_duration).count() / double(_slowdown_duration.count())))) * _prev_time_factor;
        }
        if (_controller_state == 2)
        {
            _time_factor = double(std::min(1., double(std::chrono::duration_cast<std::chrono::microseconds>(_t_current - _t_stop).count() / double(_slowdown_duration.count())))) * _prev_time_factor;
            if (_time_factor == _prev_time_factor)
            {
                _controller_state = 4;
            }
        }

        if (_controller_state == 4)
        {
            for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
            {
                if (_get_result.get()->arm_current_status.at(jnt_idx) == 11)
                {
                    RCLCPP_INFO(_node->get_logger(), "Joint %i error: %f", jnt_idx, _get_result.get()->arm_current_status.at(jnt_idx));
                }
            }
        }
        // RCLCPP_INFO(_node->get_logger(), "Time factor: %f", _time_factor);
        //get trajectory time index
        _trajectory_index = int(std::min(double(_trajectory.points.size() - 1), std::floor(float(std::chrono::duration_cast<std::chrono::microseconds>(_time_accumulator).count()) / 1000000 * _trajectory_rate)));
        RCLCPP_INFO(_node->get_logger(), "t_index: %i", _trajectory_index);
        if (_trajectory_index == _trajectory.points.size() - 1)
        {
            _t_start = std::chrono::steady_clock::now();
            _time_accumulator = std::chrono::microseconds(0);
        }

        //calculate torques
        //ID
        // RCLCPP_INFO(_node->get_logger(), "getting ID");
        _t_measure = std::chrono::steady_clock::now();
        // _q = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(_trajectory.points[_trajectory_index].positions.data(), _trajectory.points[_trajectory_index].positions.size());
        _q = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(_get_result.get()->arm_current_positions.data(), _get_result.get()->arm_current_positions.size());
        _qd = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(_trajectory.points[_trajectory_index].velocities.data(), _trajectory.points[_trajectory_index].velocities.size());
        _qdd = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(_trajectory.points[_trajectory_index].accelerations.data(), _trajectory.points[_trajectory_index].accelerations.size());
        pinocchio::rnea(model, data, _q, _qd * _time_factor, _qdd * pow(_time_factor, 2));
        _tau = data.tau;

        // RCLCPP_INFO_STREAM(node_->get_logger(), "q: "<<q_.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "qd: "<<qd_.transpose());
        // RCLCPP_INFO_STREAM(node_->get_logger(), "qdd: "<<qdd_.transpose());
        RCLCPP_INFO_STREAM(_node->get_logger(), "tau: " << _tau.transpose());

        for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
        {

            RCLCPP_INFO_STREAM(_node->get_logger(), "jnt: " << jnt_idx);
            RCLCPP_INFO_STREAM(_node->get_logger(), "pos: " << _get_result.get()->arm_current_positions[jnt_idx]);
            RCLCPP_INFO_STREAM(_node->get_logger(), "v_avg: " << _avg_vel[jnt_idx]);
            RCLCPP_INFO_STREAM(_node->get_logger(), "temp: " << _avg_temp[jnt_idx]);
            RCLCPP_INFO_STREAM(_node->get_logger(), "tq: " << set_request->torques.at(jnt_idx));
            // RCLCPP_INFO(_node->get_logger(), "getting torques for jnt %i", jnt_idx);
            //dynamic PID reconfigure
            updateParams(_pid_ctrl[jnt_idx], jnt_idx);

            //calculate torques (PID+FF)

            _error = _trajectory.points[_trajectory_index].positions[jnt_idx] - (_get_result.get()->arm_current_positions.at(jnt_idx));
            _set_torque_pid_val = _pid_ctrl[jnt_idx].getValue(_error);
            //TODO: params
            _set_torque_ff_val = _trajectory.points[_trajectory_index].velocities[jnt_idx] * _time_factor * _FFv[jnt_idx];
            _set_torque_ff_val += _trajectory.points[_trajectory_index].accelerations[jnt_idx] * pow(_time_factor, 2) * _FFa[jnt_idx];
            _set_torque_val = _tau[jnt_idx] + _set_torque_pid_val + _set_torque_ff_val;
            //RCLCPP_INFO_STREAM(node_->get_logger(), "tau: "<<tau_[jnt_idx]);

            //set_torque_val=set_torque_pid_val+set_torque_ff_val;
            
            _vel_sign = ((_trajectory.points[_trajectory_index].velocities[jnt_idx] > 0) - (_trajectory.points[_trajectory_index].velocities[jnt_idx] < 0));
            _acc_sign = ((_trajectory.points[_trajectory_index].accelerations[jnt_idx] > 0) - (_trajectory.points[_trajectory_index].accelerations[jnt_idx] < 0));

            if (abs(_error) > _error_margin)
            {
                _c_friction_comp = _torque_sign * _c_friction_val[jnt_idx];
            }
            else
            {
                _c_friction_comp = _vel_sign * _c_friction_val[jnt_idx];
            }

            // _set_torque_val += _c_friction_comp;

            _set_torque_val += compensateFriction(_trajectory.points[_trajectory_index].velocities[jnt_idx], 30, jnt_idx);
            _torque_sign = ((_set_torque_val > 0) - (_set_torque_val < 0));

            //limit torque
            if (model.effortLimit[jnt_idx] != 0)
            {
                if (_set_torque_val * _torque_sign > model.effortLimit[jnt_idx])
                    _set_torque_val = model.effortLimit[jnt_idx] * _torque_sign;
            }
            
            // TODO: min tq
            if(std::abs(_trajectory.points[_trajectory_index].velocities[jnt_idx])>0.01){
                if(std::abs(_set_torque_val)<6)
                    _set_torque_val = 6 * _torque_sign;
            }

            if (_controller_state != 1)
            {
                set_request->torques.at(jnt_idx) = _set_torque_val;
                set_request->turn_motor.at(jnt_idx) = 1;
            }
            else
            {
                set_request->torques.at(jnt_idx) = 0;
                set_request->turn_motor.at(jnt_idx) = 0;
            }
            //monitor data
            _log_msg.position[jnt_idx] = _trajectory.points[_trajectory_index].positions[jnt_idx];
            _log_msg.velocity[jnt_idx] = _trajectory.points[_trajectory_index].velocities[jnt_idx];
            _log_msg.effort[jnt_idx] = _set_torque_val;
            _log_msg.header.stamp = rclcpp::Clock().now();
        }
        //debug
        RCLCPP_INFO(_node->get_logger(), "calc time: %i", std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - _t_measure).count());
        _t_measure = std::chrono::steady_clock::now();
        _log_publisher->publish(_log_msg);
        _set_result = _set_client->async_send_request(set_request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(_node, _set_result) ==
            rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            // for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
            // {
            //     RCLCPP_INFO(node_->get_logger(), "error: %i", _set_result.get()->error.at(jnt_idx));
            //     //RCLCPP_INFO_STREAM(node_->get_logger(), "error1: " <<trajectory_pos_[trajectory_index][jnt_idx]-get_result.get()->arm_current_positions.at(jnt_idx));

            // }
        }
        else
        {
            RCLCPP_ERROR(_node->get_logger(), "Failed to call service set_torques"); // CHANGE
        }
        loop_it++;
        RCLCPP_INFO(_node->get_logger(), "set_request1: %i", std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - _t_measure).count());
        //control loop frequency
        _remaining_time = std::floor(1000000 / _trajectory_rate - std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - _t_current).count());
        // time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - _t_current).count();
        // RCLCPP_INFO(_node->get_logger(), "time_taken: %i", time);

        RCLCPP_INFO(_node->get_logger(), "time_remaining: %i", _remaining_time);
        if (_remaining_time < 0)
        {
            RCLCPP_ERROR(_node->get_logger(), "loop taking too long to execute");
        }
        std::this_thread::sleep_for(std::chrono::microseconds(_remaining_time));
    }

    for (size_t i = 0; i < measured_friction_comp.size(); i++)
    {
        set_request->torques.at(i) = 0;
        set_request->turn_motor.at(i) = 0;
    }
    _set_result = _set_client->async_send_request(set_request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(_node, _set_result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
    }
    else
    {
        RCLCPP_ERROR(_node->get_logger(), "Failed to call service set_torques"); // CHANGE
    }
    RCLCPP_INFO(_node->get_logger(), "shutting down");
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    SimpleController controller(argc, argv);

    controller.init();
    return 0;
}
