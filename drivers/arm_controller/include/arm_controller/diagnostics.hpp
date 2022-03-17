#pragma once
#include <ctime>
#include "arm_controller/base_controller.hpp"

struct DiagnosticData
{
    std::vector<double> velocity;
    std::vector<double> position;
    std::vector<double> temperature;
    std::vector<double> ma_val;
    std::vector<double> static_f_torque;
};

class Diagnostics : public BaseController
{
public:
    Diagnostics(int argc, char **argv);
    // ~Diagnostics() {}
    void init() override;

private:
    std::vector<DiagnosticData> _diag_data;
    size_t _diag_samples=314;
    double _const_torque;
    std::vector<double> _start_temp;
    int saveDiagnostics();
    int writeDiagnostics();

    //overrides
    int getAverageArmState() override;
    int communicate() override;
    int paramInit() override;
    int jointInit() override;
    int varInit(size_t joints_number) override;
    void controlLoop() override;
};