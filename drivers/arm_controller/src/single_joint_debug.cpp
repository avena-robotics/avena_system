#include "arm_controller/base_controller.hpp"

BaseController::BaseController(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // rclcpp::init(NULL,NULL);
    _node = rclcpp::Node::make_shared("base_controller");
    // helpers::commons::setLoggerLevel(_node->get_logger(),"debug");
    _watchdog = std::make_shared<helpers::Watchdog>(_node.get(), this, "system_monitor");
    status = custom_interfaces::msg::Heartbeat::RUNNING;

    //COMMUNICATION INIT
    _command_service = _node->create_service<custom_interfaces::srv::ControlCommand>("/arm_controller/commands", std::bind(&BaseController::setStateCb, this, std::placeholders::_1, std::placeholders::_2));
    _trajectory_sub = _node->create_subscription<trajectory_msgs::msg::JointTrajectory>("/trajectory", 1000, std::bind(&BaseController::setTrajectoryCb, this, std::placeholders::_1));
    _time_factor_sub = _node->create_subscription<std_msgs::msg::Float64>("/arm_controller/time_factor", 10, std::bind(&BaseController::setTimeFactorCb, this, std::placeholders::_1));
    _set_joint_states_pub = _node->create_publisher<sensor_msgs::msg::JointState>("set_joint_states", 10);
    _arm_joint_states_pub = _node->create_publisher<sensor_msgs::msg::JointState>("arm_joint_states", 10);
    _arm_joint_errors_pub = _node->create_publisher<sensor_msgs::msg::JointState>("arm_joint_errors", 10);
    _controller_state_pub = _node->create_publisher<std_msgs::msg::Int32>("/arm_controller/state", 10);
    _cartesian_error_norm_pub = _node->create_publisher<std_msgs::msg::Float64>("/arm_controller/error", 10);
    _security_trigger_sub = _node->create_subscription<std_msgs::msg::Bool>(
        "/security_trigger",
        rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&BaseController::securityTriggerStatusCb, this, std::placeholders::_1));

    _arm_interface = std::make_shared<ArmInterface>("0AA", (int)_trajectory_rate);
    std::this_thread::sleep_for(std::chrono::seconds(3));
}

BaseController::~BaseController()
{
}

void BaseController::loadFrictionChart(std::string path)
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

double BaseController::compensateFriction(double vel, double temp, int jnt_idx)
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
void BaseController::loadTrajTxt(std::string path)
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
void BaseController::updateParams(PID &pid, int joint_index)
{
    _node->get_parameter("Kp_gain_" + std::to_string(joint_index), _Kp[joint_index]);
    _node->get_parameter("Ki_gain_" + std::to_string(joint_index), _Ki[joint_index]);
    _node->get_parameter("Kd_gain_" + std::to_string(joint_index), _Kd[joint_index]);
    _node->get_parameter("FFv_gain_" + std::to_string(joint_index), _FFv[joint_index]);
    _node->get_parameter("FFa_gain_" + std::to_string(joint_index), _FFv[joint_index]);
    pid.update(_Kp[joint_index], _Ki[joint_index], _Kd[joint_index]);
}

void BaseController::setTrajectoryCb(trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    RCLCPP_INFO(_node->get_logger(), "Received trajectory");
    if (msg->joint_names.size() != _joints_number)
    {
        RCLCPP_INFO(_node->get_logger(), "Trajectory joint number does not match robot configuration");
        return;
    }
    _saved_trajectory = *msg;
}

//TODO:
void BaseController::securityTriggerStatusCb(std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_INFO(_node->get_logger(), "Received security pause trigger.");
    _controller_state = 1;
    if (_time_factor != 0.)
        _prev_time_factor = _time_factor;
    _time_factor = 0.;
    return;
}

void BaseController::setTimeFactorCb(std_msgs::msg::Float64::SharedPtr msg)
{
    if (_controller_state != 4)
    {
        _prev_time_factor = msg->data / 100.;
        RCLCPP_INFO(_node->get_logger(), "Time factor set to: %f ", _prev_time_factor);
    }
    else
    {
        _time_factor = msg->data / 100.;
        RCLCPP_INFO(_node->get_logger(), "Time factor set to: %f ", _time_factor);
    }

    return;
}

void BaseController::setStateCb(const std::shared_ptr<custom_interfaces::srv::ControlCommand::Request> request,
                                std::shared_ptr<custom_interfaces::srv::ControlCommand::Response> response)
{
    switch (request.get()->command)
    {
    //init
    case 0:
        RCLCPP_INFO(_node->get_logger(), "Received start command.");
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
        stopArm();
        response->feedback = "Received stop command.";
        break;
    //resume
    case 2:
        RCLCPP_INFO(_node->get_logger(), "Received resume command.");
        if (_controller_state == 3)
        {
            resumeArm();
            response->feedback = "Received resume command.";
        }
        else
        {
            RCLCPP_INFO(_node->get_logger(), "Controller needs to be paused before resuming.");
            response->feedback = "Controller needs to be paused before resuming.";
        }
        break;
    //pause
    case 3:
        if (_controller_state == 4)
        {
            pauseArm();
            response->feedback = "Received pause command.";
        }
        else
        {
            RCLCPP_INFO(_node->get_logger(), "Controller needs to be running before pausing.");
            response->feedback = "Controller needs to be running before pausing.";
        }
        break;
    //execute
    case 4:
    {
        RCLCPP_INFO(_node->get_logger(), "Received execute command.");

        if (_saved_trajectory.points.size() == 0)
        {
            response->feedback = "Please send trajectory before trying to execute it.";
            break;
        }

        bool in_place = true;
        for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
        {
            _error[jnt_idx] = std::abs(_saved_trajectory.points[0].positions[jnt_idx] - (_arm_status.joints[jnt_idx].position));
            std::cout << _error[jnt_idx] << std::endl;
            if (_error[jnt_idx] > _error_margin)
                in_place = false;
        }

        if (in_place)
        {
            _trajectory = _saved_trajectory;
            _controller_state = 4;
            _time_factor = _prev_time_factor;
            _t_start = std::chrono::steady_clock::now();
            _time_accumulator = std::chrono::microseconds(0);
            _t_current = std::chrono::steady_clock::now();
        }
        else
        {
            RCLCPP_INFO(_node->get_logger(), "Trajectory starting point is too far away from current end-effector position.");
        }

        response->feedback = "Received execute command.";
        break;
    }
    default:
        RCLCPP_INFO(_node->get_logger(), "Received unknown command.");
        response->feedback = "Received unknown command.";
        break;
    }
}

int BaseController::stopArm()
{
    RCLCPP_INFO(_node->get_logger(), "Received stop command.");
    _controller_state = 1;
    if (_time_factor != 0.)
        _prev_time_factor = _time_factor;
    _time_factor = 0.;
    return 0;
}

int BaseController::pauseArm()
{
    if (_controller_state != 3)
    {
        RCLCPP_INFO(_node->get_logger(), "Received pause command.");
        _controller_state = 3;
        _t_stop = std::chrono::steady_clock::now();
        if (_time_factor != 0.)
            _prev_time_factor = _time_factor;
    }
    return 0;
}

int BaseController::resumeArm()
{
    _controller_state = 2;
    _t_stop = std::chrono::steady_clock::now();
    return 0;
}

int BaseController::jointInit()
{
    //JOINT COMMUNICATION INIT
    RCLCPP_INFO(_node->get_logger(), "Starting executor");
    _exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    _exec->add_node(_node);
    RCLCPP_INFO(_node->get_logger(), "Getting arm state from CANDRIVER...");
    _arm_status = _arm_interface->getArmState();

    while (_arm_status.joints.size() != 1)
    {
        RCLCPP_ERROR(_node->get_logger(), "Received invalid arm state. Waiting ...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        _arm_status = _arm_interface->getArmState();
    }

    RCLCPP_INFO(_node->get_logger(), "Got arm state from CANDRIVER");
    _joints_number = _arm_status.joints.size();
    for (size_t i = 0; i < _joints_number; i++)
    {
        if (_arm_status.joints[i].state == 255)
        {
            RCLCPP_ERROR_STREAM(_node->get_logger(), "Current joint error: " << _arm_status.joints[i].current_error << ", previous joint error: " << _arm_status.joints[i].prev_error << " on joint " << i);
        }
    }

    RCLCPP_INFO(_node->get_logger(), "Joint number: %i", _joints_number);

    RCLCPP_INFO(_node->get_logger(), "Done initializing joints.");
    return 0;
}

int BaseController::jointPositionInit()
{
    RCLCPP_INFO(_node->get_logger(), "Initializing joint position.");
    //JOINT POSITION INIT
    for (size_t jnt_idx = 0; jnt_idx < 6; jnt_idx++)
    {
        _arm_command.joints[jnt_idx].c_status = 0;
        _arm_command.joints[jnt_idx].c_torque = 0;
    }

    for (size_t jnt_idx = 0; jnt_idx < 6; jnt_idx++)
    {
        while (_arm_status.joints[jnt_idx].state == 1)
        {
            _arm_status = _arm_interface->getArmState();
            RCLCPP_INFO(_node->get_logger(), "Joint number: %i", jnt_idx);
            std::cout << "Initializing joint " << jnt_idx << std::endl;

            //send init command to a single joint
            _arm_command.joints[jnt_idx].c_status = 2;
            _arm_command.timestamp = std::chrono::steady_clock::now();
            _arm_interface->setArmCommand(_arm_command);
            std::this_thread::sleep_for(std::chrono::microseconds((int)(std::floor(1000000 / _trajectory_rate))));
        }
        _arm_command.joints[jnt_idx].c_status = 3;
        _arm_command.timestamp = std::chrono::steady_clock::now();
        _arm_interface->setArmCommand(_arm_command);
        std::this_thread::sleep_for(std::chrono::microseconds((int)(500000)));

        _arm_command.joints[jnt_idx].c_status = 2;
        _arm_command.timestamp = std::chrono::steady_clock::now();
        _arm_interface->setArmCommand(_arm_command);
        std::this_thread::sleep_for(std::chrono::microseconds((int)(500000)));
    }
    RCLCPP_INFO(_node->get_logger(), "Done initializing robot position.");
    return 0;
}

int BaseController::idInit()
{
    RCLCPP_INFO(_node->get_logger(), "Initializing robot model.");
    //ID INIT
    _urdf = helpers::commons::getRobotDescription();
    while (_urdf.size() == 0)
    {
        RCLCPP_WARN(_node->get_logger(), "Could not get robot model, make sure it is loaded properly.");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        _urdf = helpers::commons::getRobotDescription();
    }
    pinocchio::urdf::buildModelFromXML(_urdf, _model, true);
    _data = std::make_shared<pinocchio::Data>(_model);
    _traj_data = std::make_shared<pinocchio::Data>(_model);
    RCLCPP_INFO(_node->get_logger(), "Done loading robot model.");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return 0;
}

int BaseController::paramInit()
{
    //PARAMETERS INIT
    RCLCPP_INFO(_node->get_logger(), "Initializing controller parameters.");
    _node->declare_parameter<double>("error_margin", 0.01);
    _node->declare_parameter<double>("_cartesian_error_margin", 0.01);
    _node->declare_parameter<std::string>("config_path", "");
    _node->get_parameter("config_path", _config_path);
    _node->get_parameter("error_margin", _error_margin);
    _node->get_parameter("_cartesian_error_margin", _cartesian_error_margin);
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
    return 0;
}

int BaseController::varInit()
{
    RCLCPP_INFO(_node->get_logger(), "Initializing controller variables.");
    _arm_command.joints.resize(6);
    _friction_chart.resize(_joints_number);
    _Kp.resize(_joints_number);
    _Ki.resize(_joints_number);
    _Kd.resize(_joints_number);
    _FFv.resize(_joints_number);
    _FFa.resize(_joints_number);
    _i_clamp_h.resize(_joints_number);
    _i_clamp_l.resize(_joints_number);
    _c_friction_val.resize(_joints_number);
    _error.resize(_joints_number);

    _set_joint_state_msg.name.resize(_joints_number);
    _set_joint_state_msg.position.resize(_joints_number);
    _set_joint_state_msg.velocity.resize(_joints_number);
    _set_joint_state_msg.effort.resize(_joints_number);

    _arm_joint_state_msg.name.resize(_joints_number);
    _arm_joint_state_msg.position.resize(_joints_number);
    _arm_joint_state_msg.velocity.resize(_joints_number);
    _arm_joint_state_msg.effort.resize(_joints_number);

    _arm_joint_errors_msg.name.resize(_joints_number);
    _arm_joint_errors_msg.position.resize(_joints_number);
    _arm_joint_errors_msg.velocity.resize(_joints_number);
    _arm_joint_errors_msg.effort.resize(_joints_number);

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

    //MEASUREMENT INIT
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
    }

    //TIME INIT
    _loop_it = 0;
    _t_start = std::chrono::steady_clock::now();
    _time_accumulator = std::chrono::microseconds(0);
    _controller_state = 4;
    _time_factor = 1.;
    _prev_time_factor = 0.;
    _t_current = std::chrono::steady_clock::now();
    _t_stop = std::chrono::steady_clock::now();
    _slowdown_duration = std::chrono::microseconds(1000000);

    RCLCPP_INFO(_node->get_logger(), "Done initializing variables.");
    return 0;
}
int BaseController::getAverageArmState()
{
    for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    {
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

int BaseController::handleControllerState()
{
    //STATES
    //stop
    if (_controller_state == 3)
    {
        if (_time_factor != 0.)
            _time_factor = double(std::max(0., double(std::chrono::duration_cast<std::chrono::microseconds>(_t_stop - _t_current + _slowdown_duration).count() / double(_slowdown_duration.count())))) * _prev_time_factor;
    }
    //resume
    if (_controller_state == 2)
    {
        _time_factor = double(std::min(1., double(std::chrono::duration_cast<std::chrono::microseconds>(_t_current - _t_stop).count() / double(_slowdown_duration.count())))) * _prev_time_factor;
        if (_time_factor == _prev_time_factor)
        {
            _controller_state = 4;
        }
    }
    //execute
    if (_controller_state == 4)
    {
        for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
        {
            if (_arm_status.joints[jnt_idx].state == 255)
            {
                RCLCPP_ERROR(_node->get_logger(), "Joint %i current error status: %i, previous error status: %i", jnt_idx, _arm_status.joints[jnt_idx].current_error, _arm_status.joints[jnt_idx].prev_error);
                stopArm();
            }
        }
    }
    return 0;
}

int BaseController::calculateTorque()
{
    // helpers::Timer asdf(__func__,_node->get_logger());
    for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    {
        //dynamic PID reconfigure
        // updateParams(_pid_ctrl[jnt_idx], jnt_idx);

        //calculate torques (PID+FF)
        // _set_vel = _trajectory.points[_trajectory_index].velocities[jnt_idx];
        // _error[jnt_idx] = _trajectory.points[_trajectory_index].positions[jnt_idx] - (_arm_status.joints[jnt_idx].position);

        // if (std::abs(_error[jnt_idx]) > _error_margin)
        // {
        //     // _arm_command.joints[jnt_idx].c_torque = 0;
        //     // _arm_command.joints[jnt_idx].c_status = 3;

        //     RCLCPP_WARN(_node->get_logger(), "Joint %i exceeded trajectory error margin by %f", jnt_idx, _error[jnt_idx] - _error_margin);
        //     // pauseArm();

        //     // RCLCPP_WARN(_node->get_logger(), "Joint %i exceeded trajectory error margin by %f", jnt_idx, _error[jnt_idx] - _error_margin);
        //     // _controller_state = 3;
        //     // _t_stop = std::chrono::steady_clock::now();
        //     // if (_time_factor != 0.)
        //     //     _prev_time_factor = _time_factor;

        //     // return 1;
        // }

        // _set_torque_pid_val = _pid_ctrl[jnt_idx].getValue(_error[jnt_idx]);
        // _set_torque_ff_val = _set_vel * _time_factor * _FFv[jnt_idx];
        // _set_torque_ff_val += _trajectory.points[_trajectory_index].accelerations[jnt_idx] * pow(_time_factor, 2) * _FFa[jnt_idx];

        // _vel_sign = ((_set_vel > 0) - (_set_vel < 0));
        // _acc_sign = ((_trajectory.points[_trajectory_index].accelerations[jnt_idx] > 0) - (_trajectory.points[_trajectory_index].accelerations[jnt_idx] < 0));

        // _set_torque_val = _set_torque_pid_val + compensateFriction(_set_vel * _time_factor, 25., jnt_idx);
        //DEBUG TRAJECTORY

        double arm_length = 0.55;
        double weight = 15.5;
        double gravity = std::sin(_arm_status.joints[jnt_idx].position)*arm_length*weight*9.81;
        double start_tq = 15.;
        double finish_tq = 90.;
        double debug_duration = 12.; //seconds
        double d_tq = (finish_tq - start_tq) / debug_duration * ((double)_time_accumulator.count() / 1000000.);

        _torque_sign = ((_set_torque_val > 0) - (_set_torque_val < 0));


        if (_time_accumulator.count()/1000000>=debug_duration){
            _time_accumulator = std::chrono::microseconds(0);
        }
        //
        _set_vel=std::max(0.05,std::abs(_arm_status.joints[jnt_idx].velocity))*_torque_sign*0.9;
        _set_torque_val = start_tq + d_tq + gravity + compensateFriction(_set_vel * _time_factor, 25., jnt_idx);
        // _set_torque_val = gravity;
        

        //limit torque

        if (_set_torque_val * _torque_sign > 200)
            _set_torque_val = 200 * _torque_sign;

        if (_controller_state != 1)
        {
            for (size_t i = 0; i < 6; i++)
            {
                _arm_command.joints[i].c_torque = _set_torque_val;
                _arm_command.joints[i].c_status = 3;
            }
        }
        else
        {
            for (size_t i = 0; i < 6; i++)
            {
                _arm_command.joints[i].c_torque = 0;
                //TODO: change to stop (2)
                _arm_command.joints[i].c_status = 2;
            }
        }
    }
    return 0;
}

int BaseController::calculateID()
{
    // helpers::Timer asdf(__func__,_node->get_logger());
    //ID
    std::vector<double> q_temp;
    for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    {
        q_temp.push_back(_arm_status.joints[jnt_idx].position);
    }
    _q_traj = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(_trajectory.points[_trajectory_index].positions.data(), _trajectory.points[_trajectory_index].positions.size());
    _q = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_temp.data(), q_temp.size());
    _qd = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(_trajectory.points[_trajectory_index].velocities.data(), _trajectory.points[_trajectory_index].velocities.size());
    _qdd = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(_trajectory.points[_trajectory_index].accelerations.data(), _trajectory.points[_trajectory_index].accelerations.size());
    pinocchio::rnea(_model, *_data, _q, _qd * _time_factor, _qdd * pow(_time_factor, 2));
    _tau = _data->tau;
    pinocchio::forwardKinematics(_model, *_data, _q);
    pinocchio::forwardKinematics(_model, *_traj_data, _q_traj);
    _cartesian_error_norm = (_data->oMi[6].translation() - _traj_data->oMi[6].translation()).transpose().norm();

    return 0;
}

int BaseController::communicate()
{
    // helpers::Timer asdf(__func__,_node->get_logger());
    for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    {
        //monitor data
        _set_joint_state_msg.position[jnt_idx] = 0.;
        _set_joint_state_msg.velocity[jnt_idx] = 0.;
        _set_joint_state_msg.effort[jnt_idx] = _arm_command.joints[jnt_idx].c_torque;
        _set_joint_state_msg.header.stamp = rclcpp::Clock().now();

        // _arm_joint_state_msg.name[jnt_idx] = _model.names[jnt_idx + 1]; //first object is 'universe', hence the +1
        _arm_joint_state_msg.position[jnt_idx] = _arm_status.joints[jnt_idx].position;
        _arm_joint_state_msg.velocity[jnt_idx] = _arm_status.joints[jnt_idx].velocity;
        _arm_joint_state_msg.effort[jnt_idx] = _arm_status.joints[jnt_idx].torque;
        _arm_joint_state_msg.header.stamp = rclcpp::Clock().now();

        // _arm_joint_errors_msg.position[jnt_idx] = _error[jnt_idx];
        // _arm_joint_errors_msg.header.stamp = rclcpp::Clock().now();
    }

    //comms

    std_msgs::msg::Int32 state_msg;
    std_msgs::msg::Float64 error_msg;
    state_msg.set__data(_controller_state);
    // error_msg.set__data(_cartesian_error_norm);
    _set_joint_states_pub->publish(_set_joint_state_msg);
    _arm_joint_states_pub->publish(_arm_joint_state_msg);
    // _arm_joint_errors_pub->publish(_arm_joint_errors_msg);
    // _cartesian_error_norm_pub->publish(error_msg);
    _controller_state_pub->publish(state_msg);
    _arm_command.timestamp = std::chrono::steady_clock::now();
    _arm_interface->setArmCommand(_arm_command);
    return 0;
}

void BaseController::controlLoop()
{
    //GET TIME
    _time_accumulator += std::chrono::duration_cast<std::chrono::microseconds>((std::chrono::steady_clock::now() - _t_current) * _time_factor);
    _t_current = std::chrono::steady_clock::now();

    //GET JOINT STATES
    _arm_status = _arm_interface->getArmState();

    // for (size_t i = 0; i < _joints_number; i++)
    // {
    //     if (_arm_status.joints[i].state == 255)
    //     {
    //         RCLCPP_ERROR_STREAM(_node->get_logger(), "Current joint error status: " << _arm_status.joints[i].current_error << ", previous joint error status: " << _arm_status.joints[i].prev_error << " on joint " << i);
    //     }
    // }

    // if (std::chrono::duration_cast<std::chrono::microseconds>((std::chrono::steady_clock::now() - _arm_status.timestamp)).count() > (1000000 / _trajectory_rate * 1.2))
    // {
    //     RCLCPP_WARN(_node->get_logger(), "Communication delay exceeded loop period.");
    // }

    //is this still needed?
    // getAverageArmState();
    handleControllerState();

    //get trajectory time index
    // _trajectory_index = int(std::min(double(_trajectory.points.size() - 1), std::floor(float(std::chrono::duration_cast<std::chrono::microseconds>(_time_accumulator).count()) / 1000000 * _trajectory_rate)));

    // //GO INTO HOLD IF TRAJECTORY FINISHED
    // if (_trajectory_index == _trajectory.points.size() - 1)
    // {
    //     if (_time_factor != 0.0)
    //     {
    //         RCLCPP_INFO(_node->get_logger(), "Finished trajectory");
    //         pauseArm();
    //         _time_factor = 0.;
    //     }
    // }
    // calculateID();
    calculateTorque();
    communicate();

    //execute callbacks
    _exec->spin_some();

    _loop_it++;
    //control loop frequency
    _remaining_time = std::floor(1000000 / _trajectory_rate - std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - _t_current).count());

    if (_remaining_time < 0)
    {
        RCLCPP_ERROR(_node->get_logger(), "Loop taking too long to execute. Loop took: %i us", std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - _t_current).count());
    }
    std::this_thread::sleep_for(std::chrono::microseconds(_remaining_time));
}

//initialize movement functionalities, start controller
void BaseController::init()
{

    jointInit();
    // idInit();
    varInit();
    paramInit();
    // jointPositionInit();

    //CONTROL LOOP
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    RCLCPP_INFO(_node->get_logger(), "Done initializing, entering control loop");

    while (rclcpp::ok())
    {
        controlLoop();
    }

    for (size_t jnt_idx = 0; jnt_idx < 6; jnt_idx++)
    {
        _arm_command.joints[jnt_idx].c_torque = 0;
        //TODO: change to stop (2)
        _arm_command.joints[jnt_idx].c_status = 2;
    }
    _arm_command.timestamp = std::chrono::steady_clock::now();
    _arm_interface->setArmCommand(_arm_command);
    RCLCPP_INFO(_node->get_logger(), "shutting down");
    exit(0);
}

int main(int argc, char **argv)
{
    BaseController controller(argc, argv);

    controller.init();
    return 0;
}
