#pragma once
#include <float.h>
#include <vector>

class PID
{
public:
    PID(double Kp, double Ki, double Kd, double dt, int d_n);
    PID(double Kp, double Ki, double Kd, double dt, double i_clamp_low, double i_clamp_high, int d_n);
    ~PID();
    double getValue(double error);
    void update(double Kp, double Ki, double Kd);

private:
    std::vector<double> _d_buffer;
    double Kp_, Ki_, Kd_, dt_, i_clamp_low_, i_clamp_high_;
    double i_val_, prev_error_, _d_sum;
    int _d_n, _iter;
};