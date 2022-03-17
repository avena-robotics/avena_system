#pragma once

#include "custom_interfaces/srv/control_command.hpp"
#include "custom_interfaces/srv/set_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

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

class TrajectoryFeeder : public rclcpp::Node
{
public:
    TrajectoryFeeder();


private:
    void sendArmCommand(int command);
    void _saveTrajectoryCb(trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void _controllerStateCb(std_msgs::msg::Int32::SharedPtr msg);
    void _startFeedingCb(std_msgs::msg::Int32::SharedPtr msg);

    void feedTrajectories();

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _controller_state_sub, _start_feeding_sub;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr _trajectory_sub;

    rclcpp::Client<custom_interfaces::srv::ControlCommand>::SharedPtr _arm_command_client_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _trajectory_pub;

    std::vector<trajectory_msgs::msg::JointTrajectory> _trajectory_array;
    rclcpp::TimerBase::SharedPtr _loop_timer;
    int _traj_it = 0;
    int _controller_state = 0;
    int _feed=0;
};