#pragma once

#ifndef BASE_CONTROLLER_HPP_
#define BASE_CONTROLLER_HPP_

#include "custom_interfaces/srv/set_arm_torques.hpp"
#include "custom_interfaces/srv/get_arm_state.hpp"
#include "custom_interfaces/srv/control_command.hpp"
#include "custom_interfaces/srv/set_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

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
#include <filesystem>

#include "PID.hpp"
#include "hw_interface.hpp"
#include "helpers_commons/helpers_commons.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
// #include "pinocchio/algorithm/parallel/rnea.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

struct friction_comp
{
    double vel;
    double tq;
    double temp;
};

class BaseController : public helpers::WatchdogInterface
{
public:
    // BaseController();
    BaseController(int argc, char **argv);
    ~BaseController();

    //initialize all controller components, start control loop
    virtual void init();

    void initNode() override
    {
        status = custom_interfaces::msg::Heartbeat::RUNNING;
    };
    void shutDownNode() override
    {
        status = custom_interfaces::msg::Heartbeat::STOPPED;
        rclcpp::shutdown();
    };

    std::shared_ptr<rclcpp::Node> _node;

protected:
    bool getArmState();
    bool setArmCommand();

    //loads friction chart
    void loadFrictionChart(std::string path);
    //updates PID parameters
    void updateParams(PID &pid, int joint_index);
    //reads friction value from loaded friction chart, corresponding to current join velocity and temperature
    double compensateFriction(double vel, double temp, int jnt_idx);

    virtual int idInit();
    virtual int paramInit();
    virtual int varInit();
    virtual int jointInit();
    virtual int jointPositionInit();
    virtual int getAverageArmState();
    virtual int handleControllerState();
    virtual int calculateTorque();
    virtual int calculateID();
    virtual int communicate();

    virtual void controlLoop();

    int stopArm();
    int pauseArm();
    int resumeArm();

    void setStateCb(const std::shared_ptr<custom_interfaces::srv::ControlCommand::Request> request,
                    std::shared_ptr<custom_interfaces::srv::ControlCommand::Response> response);
    void setTimeFactorCb(std_msgs::msg::Float64::SharedPtr msg);
    void setTrajectoryCb(trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void securityTriggerStatusCb(std_msgs::msg::Bool::SharedPtr);

    helpers::Watchdog::SharedPtr _watchdog;

    // std::shared_ptr<ArmInterface> _arm_interface;

    //PARAMETERS
    size_t _joints_number = 6;
    double _error_margin, _cartesian_error_margin, _communication_rate, _trajectory_rate;
    std::string _config_path;
    std::string _urdf;

    //PID
    std::vector<double> _Kp;
    std::vector<double> _Ki;
    std::vector<double> _Kd;
    std::vector<double> _FFv;
    std::vector<double> _FFa;
    std::vector<double> _c_friction_val;
    std::vector<double> _i_clamp_h;
    std::vector<double> _i_clamp_l;

    std::vector<PID> _pid_ctrl;

    //CONTROL
    double _set_torque_val, _set_torque_ff_val, _set_torque_pid_val, _cartesian_error_norm, _c_friction_comp, _set_vel;
    std::vector<double> _error;
    int _torque_sign, _vel_sign, _time, _remaining_time, _acc_sign;
    size_t _trajectory_index;
    int _controller_state;
    std::vector<int> _jitter_counter, _jitter_threshold;
    std::vector<double> _jitter_multiplier;
    std::vector<bool> _jitter_present;

    std::shared_ptr<boost::interprocess::managed_shared_memory> _shared_memory_segment;

    std::shared_ptr<ArmStatus> _shm_arm_status;
    std::shared_ptr<ArmCommand> _shm_arm_command;

    ArmStatus _arm_status;
    ArmCommand _arm_command;

    //MEASUREMENT
    const size_t _avg_samples = 200;
    std::vector<double> _avg_temp, _avg_vel, _avg_acc, _avg_tau, _avg_pos, _prev_pos;
    //buffers
    std::vector<std::vector<double>> _avg_temp_b, _avg_vel_b, _avg_acc_b, _avg_tau_b, _avg_pos_b;
    std::vector<double> _frick_acu;

    //FRICTION
    std::vector<std::vector<std::vector<friction_comp>>> _friction_chart;
    std::vector<std::vector<friction_comp>> _measured_friction_comp;

    //KINEMATICS

    //ID
    Eigen::VectorXd _q, _qd, _qdd, _tau, _q_traj;
    pinocchio::Model _model;
    std::shared_ptr<pinocchio::Data> _data, _traj_data;

    //TIME
    rclcpp::TimerBase::SharedPtr _timer;
    std::chrono::time_point<std::chrono::steady_clock> _t_start, _t_stop, _t_current, _t_measure;
    std::chrono::microseconds _time_accumulator, _slowdown_duration;
    double _time_factor, _prev_time_factor;
    int _loop_it;

    //TRAJECTORY
    trajectory_msgs::msg::JointTrajectory _trajectory;
    trajectory_msgs::msg::JointTrajectory _saved_trajectory;


    //ROS
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> _exec;
    rclcpp::callback_group::CallbackGroup::SharedPtr _cb_group;

    //COMMUNICATION

    //__PUBLISHERS
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _set_joint_states_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _arm_joint_states_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _arm_joint_errors_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _cartesian_error_norm_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _controller_state_pub;

    //__SUBSCRIBERS
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _security_trigger_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _time_factor_sub;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr _trajectory_sub;

    //__SERVICES
    rclcpp::Service<custom_interfaces::srv::ControlCommand>::SharedPtr _command_service;

    sensor_msgs::msg::JointState _set_joint_state_msg, _arm_joint_state_msg, _arm_joint_errors_msg;
};

#endif // BASE_CONTROLLER_HPP_