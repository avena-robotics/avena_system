#include "arm_controller/simple_controller.hpp"

using namespace std::chrono_literals;

//TODO: init essential functionalities, get ready to start
SimpleController::SimpleController(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    _node = rclcpp::Node::make_shared("simple_controller");
    _command_service = _node->create_service<custom_interfaces::srv::ControlCommand>("/arm_controller/command", std::bind(&SimpleController::setStateCb, this, std::placeholders::_1, std::placeholders::_2));
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
    return _friction_chart[jnt_idx][v][t].tq;
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
void SimpleController::setStateCb(const std::shared_ptr<custom_interfaces::srv::ControlCommand::Request> request,
                                  std::shared_ptr<custom_interfaces::srv::ControlCommand::Response> response)
{
    switch (request.get()->command)
    {
    //stop
    case 0:
        RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Received initialization command.");
        _controller_state = 0; //stops control loop
        //engage brakes
        break;

    //init
    case 1:
        RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Received stop command.");
        _controller_state = 1;
        _t_stop = std::chrono::steady_clock::now();
        //starts init and control loop with 0 vel,acc - holds position
        break;
    //resume
    case 2:
        RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Received resume command.");
        _controller_state = 2;
        //(smoothly?) resumes previous trajectory
        break;
    //pause
    case 3:
        RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Received pause command.");
        _controller_state = 3;
        _t_stop = std::chrono::steady_clock::now();

        //(smoothly?) pauses current trajectory
        //saves current trajectory progress
        //executes hold position trajectory
        break;
    //execute
    case 4:
        RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Received execute command.");
        _controller_state = 4;
        //starts executing current trajectory
        break;
    default:
        RCLCPP_INFO(rclcpp::get_logger("simple_controller"), "Received unknown command.");
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

    //get joints
    _get_result = _get_client->async_send_request(get_request);

    if (rclcpp::spin_until_future_complete(_node, _get_result) == rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        _joints_number = _get_result.get()->arm_current_status.size();
        for (size_t i; i < _joints_number; i++)
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

    set_request->torques.resize(_joints_number);
    set_request->turn_motor.resize(_joints_number);

    _friction_chart.resize(_joints_number);
    _Kp.resize(_joints_number);
    _Ki.resize(_joints_number);
    _Kd.resize(_joints_number);
    _FFv.resize(_joints_number);
    _FFa.resize(_joints_number);
    _c_friction_val.resize(_joints_number);
    _i_clamp_h.resize(_joints_number);
    _i_clamp_l.resize(_joints_number);
    measured_friction_comp.resize(_joints_number);

    // std::string urdf_path = "/root/ros2_ws/src/avena_ros2_control/avena_bringup/urdf/v4.urdf.xacro";

    // //initialize ID
    // RCLCPP_INFO_STREAM(_node->get_logger(), "Loading robot model from: " << urdf_path);
    // pinocchio::Model model;
    // pinocchio::urdf::buildModel(urdf_path, model, true);

    // pinocchio::Data data(model);

    // _q.resize(_joints_number);
    // _qd.resize(_joints_number);
    // _qdd.resize(_joints_number);

    // RCLCPP_INFO(_node->get_logger(), "Done loading robot model.");

    //parameters
    _node->declare_parameter<double>("error_margin", 0);
    _node->get_parameter("error_margin", _error_margin);
    _node->declare_parameter<std::string>("config_path", "");
    _node->get_parameter("config_path", _config_path);
    //fck ros2
    _config_path = "/home/avena/avena_system/src/avena_system/drivers/arm_controller/config";
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
        _pid_ctrl.push_back(PID(_Kp[i], _Ki[i], _Kd[i], 1.0 / _trajectory_rate, _i_clamp_l[i], _i_clamp_h[i], _avg_samples));
    }

    //initialize log topic
    _log_msg.position.resize(_joints_number);
    _log_msg.velocity.resize(_joints_number);
    _log_msg.effort.resize(_joints_number);
    _set_joint_states_pub = _node->create_publisher<sensor_msgs::msg::JointState>("/set_joint_states", 10);
    RCLCPP_INFO(_node->get_logger(), "dupa00");
    //load trajectory from txt (for testing purposes)
    loadTrajTxt("/home/avena/ros2_ws/src/avena_ros2_control/arm_controller/zero_trajectory/");
    loadFrictionChart(_config_path + std::string("/friction_chart_"));
    RCLCPP_INFO(_node->get_logger(), "dupa0");
    // for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
    // {

    //     set_request->torques.at(jnt_idx) = 0;
    //     set_request->turn_motor.at(jnt_idx) = 1;
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
        for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
        {
            measured_friction_comp[jnt_idx].clear();
            int file_i = 0;
            std::string filename = std::string("/f_chart_joint_") + std::to_string(jnt_idx);
            vel_achi[jnt_idx] = false;
            cal_done[jnt_idx] = false;
            valid_vel_time[jnt_idx] = std::chrono::steady_clock::now();
            tq_acc[jnt_idx] = 0;
            tq_inc[jnt_idx] = 0;
            goal_v_avg_acc[jnt_idx] = 0;
            set_vel[jnt_idx] = 0.8;
            prev_pos[jnt_idx] = _get_result.get()->arm_current_positions[jnt_idx];
            while (std::filesystem::exists(std::filesystem::path(_config_path + filename + std::string("_") + std::to_string(file_i) + std::string(".txt"))))
                file_i++;
            std::cout << _config_path + filename + std::string("_") + std::to_string(file_i) + std::string(".txt") << std::endl;
            //RCLCPP_INFO_STREAM(_node->get_logger(), "Chart file: " << (std::string("/root/temp/t_charts/chart_") + std::string("joint") + std::to_string(jnt_idx) + std::string("_") + std::to_string(dir) + std::string(".txt")));
            chart[jnt_idx] = std::make_shared<std::ofstream>(_config_path + filename + std::string("_") + std::to_string(file_i) + std::string(".txt"));
            // chart[jnt_idx] = std::make_shared<std::ofstream>(std::string("/home/avena/ros2_ws/src/avena_ros2_control/arm_controller/config/f_chart_.txt"));

            for (int i = 0; i < n; i++)
            {
                p_avg[jnt_idx][i] = _get_result.get()->arm_current_positions[jnt_idx];
                temp_avg[jnt_idx][i] = 0;
                tau_avg[jnt_idx][i] = 0;
            }
            chart[jnt_idx]->precision(6);
            //*chart[jnt_idx] << "position:\tvel:\ttemp:\ttorque_read:\ttorque_set:" << std::endl;
            //TODO: static friction pls
            // *chart[jnt_idx] << "0.00 0.0" << std::endl;
        }

        int loop_it = 0;

        _t_start = std::chrono::steady_clock::now();
        RCLCPP_INFO(_node->get_logger(), "dupa2");
        //TODO: change trajectory/time relation
        //loop

        while (rclcpp::ok())
        {

            //std::chrono::time_point<std::chrono::steady_clock> t_current = std::chrono::steady_clock::now();

            _get_result = _get_client->async_send_request(get_request);
            auto t_current = std::chrono::steady_clock::now();
            if (_controller_state == 3)
            {
            }
            if (rclcpp::spin_until_future_complete(_node, _get_result) == rclcpp::executor::FutureReturnCode::SUCCESS)
            {
                for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
                {

                    //crimge
                    if (std::abs(_get_result.get()->arm_current_positions.at(jnt_idx) - prev_pos[jnt_idx]) > 10)
                    {
                        for (size_t i = 0; i < n; i++)
                        {
                            p_avg[jnt_idx][i] = _get_result.get()->arm_current_positions.at(jnt_idx);
                        }
                        prev_pos[jnt_idx] = _get_result.get()->arm_current_positions.at(jnt_idx);
                    }
                    p_avg[jnt_idx][loop_it % n] = _get_result.get()->arm_current_positions.at(jnt_idx);

                    temp_avg[jnt_idx][loop_it % n] = _get_result.get()->arm_current_temperature.at(jnt_idx);
                    tau_avg[jnt_idx][loop_it % n] = _get_result.get()->arm_current_torques.at(jnt_idx);

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
            }
            else
            {
                RCLCPP_ERROR(_node->get_logger(), "Failed to call service get_arm_state"); // CHANGE
                rclcpp::shutdown();
                return;
            }
            //calculate torques
            for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
            {
                if (cal_done[jnt_idx])
                {
                    set_request->torques.at(jnt_idx) = 0;
                    set_request->turn_motor.at(jnt_idx) = 0;
                    continue;
                }
                //dynamic PID reconfigure
                updateParams(_pid_ctrl[jnt_idx], jnt_idx);

                //calculate torques (PID+FF)

                _error = set_vel[jnt_idx] - v_avg[jnt_idx];
                _set_torque_val = _pid_ctrl[jnt_idx].getValue(_error) + compensateFriction(set_vel[jnt_idx], _get_result.get()->arm_current_temperature[jnt_idx], jnt_idx);
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
                                measured_friction_comp[jnt_idx].push_back(temp_fc);
                            }
                            temp_fc.tq = tq_acc[jnt_idx] / double(tq_inc[jnt_idx]);
                            temp_fc.vel = goal_v_avg_acc[jnt_idx] / double(tq_inc[jnt_idx]);
                            temp_fc.temp = temp_avg_s[jnt_idx];
                            measured_friction_comp[jnt_idx].push_back(temp_fc);
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

                // set_request->torques.at(jnt_idx) = _set_torque_val;
                set_request->torques.at(jnt_idx) = -7;
                set_request->turn_motor.at(jnt_idx) = 1;
                RCLCPP_INFO_STREAM(_node->get_logger(), "jnt: " << jnt_idx);
                RCLCPP_INFO_STREAM(_node->get_logger(), "pos: " << _get_result.get()->arm_current_positions[jnt_idx]);
                RCLCPP_INFO_STREAM(_node->get_logger(), "set_vel: " << set_vel[jnt_idx]);
                RCLCPP_INFO_STREAM(_node->get_logger(), "v_avg: " << v_avg[jnt_idx]);
                RCLCPP_INFO_STREAM(_node->get_logger(), "temp: " << temp_avg_s[jnt_idx]);
                RCLCPP_INFO_STREAM(_node->get_logger(), "tq: " << set_request->torques.at(jnt_idx));

                if (vel_achi[jnt_idx] && (std::chrono::steady_clock::now() - valid_vel_time[jnt_idx] > std::chrono::seconds(1)))
                {
                    // *chart[jnt_idx] << p_avg_s[jnt_idx] << '\t' << v_avg[jnt_idx] << '\t' << temp_avg_s[jnt_idx] << '\t' << tau_avg_s[jnt_idx] << '\t' << set_vel << std::endl;
                    tq_acc[jnt_idx] += _set_torque_val;
                    tq_inc[jnt_idx]++;
                    goal_v_avg_acc[jnt_idx] += v_avg[jnt_idx];
                }
                //monitor data
                _log_msg.position[jnt_idx] = v_avg[jnt_idx];
                _log_msg.velocity[jnt_idx] = set_vel[jnt_idx];
                _log_msg.effort[jnt_idx] = _set_torque_val;
                _log_msg.header.stamp = rclcpp::Clock().now();
            }
            //debug
            RCLCPP_INFO(_node->get_logger(), "calc time: %i", std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t_current).count());
            _set_joint_states_pub->publish(_log_msg);
            _set_result = _set_client->async_send_request(set_request);
            // Wait for the result.
            if (rclcpp::spin_until_future_complete(_node, _set_result) ==
                rclcpp::executor::FutureReturnCode::SUCCESS)
            {
                // for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
                // {
                //     RCLCPP_INFO(node_->get_logger(), "error: %i", result.get()->error.at(jnt_idx));
                //     //RCLCPP_INFO_STREAM(node_->get_logger(), "error1: " <<trajectory_pos_[trajectory_index][jnt_idx]-get_result.get()->arm_current_positions.at(jnt_idx));

                // }
            }
            else
            {
                RCLCPP_ERROR(_node->get_logger(), "Failed to call service set_torques"); // CHANGE
            }
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
        std::cout << measured_friction_comp.size() << std::endl;

        for (size_t i = 0; i < measured_friction_comp.size(); i++)
        {
            set_request->torques.at(i) = 0;
            set_request->turn_motor.at(i) = 0;
            for (size_t j = 0; j < measured_friction_comp[i].size(); j++)
            {
                std::cout << measured_friction_comp[i].size() << std::endl;
                std::cout << "writing" << std::endl;
                *chart[i] << std::to_string(measured_friction_comp[i][j].vel) << " " << std::to_string(measured_friction_comp[i][j].tq) << " " << std::to_string(measured_friction_comp[i][j].temp) << std::endl;
            }
            chart[i]->flush();
            chart[i]->close();
        }
    }
    _set_result = _set_client->async_send_request(set_request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(_node, _set_result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        // for (size_t jnt_idx = 0; jnt_idx < _joints_number; jnt_idx++)
        // {
        //     RCLCPP_INFO(node_->get_logger(), "error: %i", result.get()->error.at(jnt_idx));
        //     //RCLCPP_INFO_STREAM(node_->get_logger(), "error1: " <<trajectory_pos_[trajectory_index][jnt_idx]-get_result.get()->arm_current_positions.at(jnt_idx));

        // }
    }
    else
    {
        RCLCPP_ERROR(_node->get_logger(), "Failed to call service set_torques"); // CHANGE
    }
    RCLCPP_INFO(_node->get_logger(), "Done calibrating, shutting down");

    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    SimpleController controller(argc, argv);

    controller.init();
    return 0;
}
