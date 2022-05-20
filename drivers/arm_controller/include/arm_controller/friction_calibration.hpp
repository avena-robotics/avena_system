#pragma once
#include "arm_controller/base_controller.hpp"



class FrictionCalibration : public BaseController
{
public:
    FrictionCalibration(int argc, char **argv);
    ~FrictionCalibration() {}
    void init() override;

private:
    std::string _serial_number;
    //overrides
    int jointInit() override;
    // int communicate() override;
    int paramInit() override;
    // int idInit() override;
    int varInit(size_t joints_number) override;
    // void controlLoop() override;

    double _command_multiplier = 2./360.;
};
