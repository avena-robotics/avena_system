#pragma once
#include "custom_interfaces/srv/set_arm_torques.hpp"
#include "custom_interfaces/srv/get_arm_state.hpp"
#include "custom_interfaces/srv/control_command.hpp"
#include "custom_interfaces/srv/set_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "PID.hpp"

#include "helpers_commons/helpers_commons.hpp"


#include <chrono>
#include <functional>
#include <string>
#include <cstdlib>
#include <memory>
#include <vector>
#include <array>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"





struct friction_comp{
    double vel;
    double tq;
};

class SimpleController : public helpers::WatchdogInterface
{
    public:
        std::shared_ptr<rclcpp::Node> _node;
        SimpleController();
        SimpleController(int argc,char **argv);
        ~SimpleController();
        void init();

        void initNode() override {
                status=custom_interfaces::msg::Heartbeat::RUNNING;
        };
        void shutDownNode() override {
                status=custom_interfaces::msg::Heartbeat::STOPPED;
        };

    private:
        //std::vector<trajectory_point> trajectory_;
        
        helpers::Watchdog::SharedPtr _watchdog;

        //TODO:
        const double _trajectory_rate=250;


        //parameters
        int _joints_number;
        double _error_margin;
        std::string _config_path;
        std::vector<double> _Kp;
        std::vector<double> _Ki;
        std::vector<double> _Kd;
        std::vector<double> _FFv;
        std::vector<double> _FFa;
        std::vector<double> _c_friction_val;
        std::vector<double> _i_clamp_h;
        std::vector<double> _i_clamp_l;

        std::vector<PID> _pid_ctrl;

        double _set_torque_val,_set_torque_ff_val,_set_torque_pid_val, _error, _c_friction_comp;
        int _trajectory_index;
        int _torque_sign, _vel_sign, _time, _remaining_time;
        int _controller_state;

        std::vector<std::vector<friction_comp>>_friction_chart;
        std::vector<std::vector<friction_comp>> measured_friction_comp;

        Eigen::VectorXd _q, _qd, _qdd, _tau;

        rclcpp::TimerBase::SharedPtr _timer;
        std::chrono::time_point<std::chrono::steady_clock> _t_start, _t_stop, _t_current,_t_measure;
        std::chrono::microseconds _time_accumulator, _slowdown_duration; 
        double _time_factor,_prev_time_factor;
        
        //communication

        //change to comply with execute_move
        //services
        rclcpp::Service<custom_interfaces::srv::ControlCommand>::SharedPtr _command_service;
        rclcpp::Service<custom_interfaces::srv::SetTrajectory>::SharedPtr _trajectory_service;

        //publishers
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _log_publisher;

        //clients
        rclcpp::Client<custom_interfaces::srv::SetArmTorques>::SharedPtr _set_client;
        rclcpp::Client<custom_interfaces::srv::GetArmState>::SharedPtr _get_client;

        std::shared_future<std::shared_ptr<custom_interfaces::srv::SetArmTorques_Response>> _set_result;
        std::shared_future<std::shared_ptr<custom_interfaces::srv::GetArmState_Response>> _get_result;

        sensor_msgs::msg::JointState _log_msg;
        trajectory_msgs::msg::JointTrajectory _trajectory;
        
        trajectory_msgs::msg::JointTrajectory _saved_trajectory;

        //methods

        void loadFrictionChart(std::string path);
        void updateParams(PID &pid, int joint_index);
        void loadTrajTxt(std::string path);
        double compensateFriction(double vel,int jnt_idx);
        void setCommand();
        void setTrajectory(const std::shared_ptr<custom_interfaces::srv::SetTrajectory::Request> request,
            std::shared_ptr<custom_interfaces::srv::SetTrajectory::Response>      response);
        void setState(const std::shared_ptr<custom_interfaces::srv::ControlCommand::Request> request,
            std::shared_ptr<custom_interfaces::srv::ControlCommand::Response> response);
};