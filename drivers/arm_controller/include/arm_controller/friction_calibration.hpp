#pragma once
#include "arm_controller/base_controller.hpp"



class FrictionCalibration : public BaseController
{
public:
    FrictionCalibration(int argc, char **argv);
    ~FrictionCalibration() {}
    void init() override;

private:

    //overrides
    // int communicate() override;
    // int paramInit() override;
    // int idInit() override;
    // int varInit() override;
    // void controlLoop() override;
};