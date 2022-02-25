#pragma once
#include "arm_controller/base_controller.hpp"

class SingleJointDebug : public BaseController
{
public:
    SingleJointDebug(int argc, char **argv);
    ~SingleJointDebug();
    void init() override;

private:

    int time_steps, stop_time;
    double _trajectory_period, _sin_amp;
    bool p_f;
    double p_q;
    double _mass_x, _mass;
    std::shared_ptr<std::ofstream> _stat_file;
    double _run_mse, _run_max, _run_mean, _stop_mse, _stop_max, _stop_mean;
    std::vector<double> _run_data, _stop_data;

    double get_mse(std::vector<double> data);
    double get_max(std::vector<double> data);
    double get_mean(std::vector<double> data);



    //overrides
    int communicate() override;
    int paramInit() override;
    int jointInit() override;
    int varInit(size_t joints_number) override;
    void controlLoop() override;
};