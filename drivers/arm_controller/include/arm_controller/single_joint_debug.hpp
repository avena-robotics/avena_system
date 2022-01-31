#pragma once
#include "arm_controller/base_controller.hpp"

class SingleJointDebug : public BaseController
{
public:
    SingleJointDebug(int argc, char **argv);
    ~SingleJointDebug() {}
    void init() override;

private:

    double _trajectory_period, _sin_amp;

    //overrides
    int communicate() override;
    int paramInit() override;
    int jointInit() override;
    int varInit(size_t joints_number) override;
    void controlLoop() override;
};