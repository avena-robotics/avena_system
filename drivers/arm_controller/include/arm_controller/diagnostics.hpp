#pragma once
#include "arm_controller/base_controller.hpp"

struct DiagnosticData
{
    std::vector<double> velocity;
    std::vector<double> position;
    std::vector<double> temperature;
    std::vector<double> static_f_torque;
};

class Diagnostics : public BaseController
{
public:
    Diagnostics(int argc, char **argv);
    ~Diagnostics() {}
    void init() override;

private:
    std::vector<DiagnosticData> _diag_data;
    size_t _diag_samples=3600;
    int saveDiagnostics();
    int writeDiagnostics();

    //overrides
    int communicate() override;
    int paramInit() override;
    int idInit() override;
    int varInit() override;
    void controlLoop() override;
};