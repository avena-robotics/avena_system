#ifndef PLUGIN_CONTROL_GRIPPER__COMMONS_HPP
#define PLUGIN_CONTROL_GRIPPER__COMMONS_HPP

#include <iostream>
#include <string>
#include <vector>
#include <memory>

using Handle = int;

struct SceneInfo
{
    void updatePrefixForNames(const std::string &prefix)
    {
        placeholder_name = prefix + "_" + placeholder_name;
        calibration_mat_name = prefix + "_" + calibration_mat_name;
    }

    Handle connection_handle;
    Handle placeholder_handle;
    Handle calibration_mat_handle;

    std::string placeholder_name = "gripper_placeholder";
    std::string calibration_mat_name = "calibration_mat";
};

struct CalibrationMatDims
{
    float x_dim;
    float y_dim;
    float z_dim;
};

struct ToolPose
{
    ToolPose()
    {
        position.resize(3);
        orientation.resize(3);
    }
    std::vector<float> position;
    std::vector<float> orientation;

    using SharedPtr = std::shared_ptr<ToolPose>;
};

#endif // PLUGIN_CONTROL_GRIPPER__COMMONS_HPP
