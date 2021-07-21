#include "grippers/franka_gripper.hpp"

FrankaGripper::FrankaGripper(const InputOptions &options)
    : Gripper(options)
{
    _left_joint_name = _input_options.prefix + "_" + _left_joint_name;
    _right_joint_name = _input_options.prefix + "_" + _right_joint_name;

    _left_joint_handle = simGetObjectHandle(_left_joint_name.c_str());
    _right_joint_handle = simGetObjectHandle(_right_joint_name.c_str());
}

int FrankaGripper::open(const OpenGripperOptions &options)
{
    if (simSetJointPosition(_left_joint_handle, 0.04) == -1)
        return 1;
    if (simSetJointPosition(_right_joint_handle, 0.04) == -1)
        return 1;
    return 0;
}

int FrankaGripper::close(const CloseGripperOptions &options)
{
    if (simSetJointPosition(_left_joint_handle, 0.0) == -1)
        return 1;
    if (simSetJointPosition(_right_joint_handle, 0.0) == -1)
        return 1;
    return 0;
}
