#ifndef PLUGIN_CONTROL_GRIPPER_HPP
#define PLUGIN_CONTROL_GRIPPER_HPP

// ___Coppelia___
#include "config.h"
#include "simPlusPlus/Plugin.h"
#include "stubs.h"

// ___CPP___
#include <memory>
#include <cmath>
#include <iostream>
#include <chrono>
#include <fstream>

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

// ___Avena___
#include <helpers_commons/helpers_commons.hpp>
#include <helpers_vision/helpers_vision.hpp>
#include <custom_interfaces/srv/change_tool.hpp>

// ___Package___
#include "commons.hpp"
#include "grippers/franka_gripper.hpp"

#define PLUGIN_NAME "AvenaControlGripper"
#define PLUGIN_VERSION 1

using ChangeToolSrv = custom_interfaces::srv::ChangeTool;
using OpenGripperSrv = std_srvs::srv::Trigger;
using CloseGripperSrv = std_srvs::srv::Trigger;

class ControlGripper : public sim::Plugin,
                       public rclcpp::Node
{
public:
    ControlGripper();
    virtual ~ControlGripper();

    void onStart() override;
    void onInstancePass(const sim::InstancePassFlags &flags) override;

private:
    int _readParametersFromServer();
    void _changeToolRequest(const std::shared_ptr<ChangeToolSrv::Request> request, std::shared_ptr<ChangeToolSrv::Response> response);
    void _openGripperRequest(const std::shared_ptr<OpenGripperSrv::Request> request, std::shared_ptr<OpenGripperSrv::Response> response);
    void _closeGripperRequest(const std::shared_ptr<CloseGripperSrv::Request> request, std::shared_ptr<CloseGripperSrv::Response> response);
    int _isToolNameValid(const std::string &tool_name);
    Handle _getDummyHandle(const std::string &name);
    Handle _createCalibrationMat(const std::string &calibration_mat_name, const CalibrationMatDims &calibration_mat_dims, const Handle &parent_handle);
    void _prepareScene();
    int _detachCurrentTool(const std::string &request_tool_name, ToolPose &tool_pose);
    void _attachTool(const std::string &name, const ToolPose &tool_pose);
    Gripper::SharedPtr _getGripper(const std::string &gripper_name);

    rclcpp::Service<ChangeToolSrv>::SharedPtr _change_tool_srv;
    rclcpp::Service<OpenGripperSrv>::SharedPtr _open_gripper_srv;
    rclcpp::Service<CloseGripperSrv>::SharedPtr _close_gripper_srv;

    CalibrationMatDims _calibration_mat_dims;
    SceneInfo _scene_info;
    Gripper::SharedPtr _gripper;

    helpers::commons::RobotInfo _robot_info;
    bool _parameters_read;
    bool _scene_prepared;
    helpers::Watchdog::SharedPtr _watchdog;

    const std::vector<std::string> _tool_names = {"calibration_mat", "gripper"};
    const float _calibration_mat_dims_safety = 0.03;
};

#endif // PLUGIN_CONTROL_GRIPPER_HPP
