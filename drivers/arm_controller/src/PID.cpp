#include <arm_controller/PID.hpp>

PID::PID(double Kp, double Ki, double Kd, double dt, int d_n)
{
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
    dt_ = dt;
    i_val_ = 0;
    prev_error_ = 0;
    i_clamp_high_ = DBL_MAX;
    i_clamp_low_ = -DBL_MAX;
    _d_n = d_n;
    _d_buffer.resize(d_n);
    _iter = 0;
    for (size_t i = 0; i > d_n; i++)
    {
        _d_buffer[i] = 0.;
    }
}

PID::PID(double Kp, double Ki, double Kd, double dt, double i_clamp_low, double i_clamp_high, int d_n)
{
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
    dt_ = dt;
    if (Ki != 0)
    {
        i_clamp_low_ = i_clamp_low / Ki;
        i_clamp_high_ = i_clamp_high / Ki;
    }
    else
    {
        i_clamp_high_ = DBL_MAX;
        i_clamp_low_ = -DBL_MAX;
    }
    i_val_ = 0;
    prev_error_ = 0;
    _d_n = d_n;
    _d_buffer.resize(d_n);
    _iter = 0;
    for (size_t i = 0; i > d_n; i++)
    {
        _d_buffer[i] = 0.;
    }
}

PID::~PID() {}

void PID::update(double Kp, double Ki, double Kd, int d_n)
{

    i_val_=i_val_/Ki*Ki_;

    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;

    if (d_n != _d_buffer.size())
    {
        _d_n=d_n;
        _iter=0;
        _d_buffer.resize(d_n);
        for (size_t i = 0; i > d_n; i++)
        {
            _d_buffer[i] = 0.;
        }
    }
}
std::array<double, 3> PID::getComponents()
{
    return std::array<double, 3>{_error_p, _error_i, _error_d};
}

double PID::getValue(double error)
{

    _d_buffer[_iter] = error;

    _error_p = error * Kp_;
    i_val_ += (error * dt_);

    if (i_val_ > i_clamp_high_)
    {
        i_val_ = i_clamp_high_;
    }
    if (i_val_ < i_clamp_low_)
    {
        i_val_ = i_clamp_low_;
    }

    _error_i = i_val_ * Ki_;

    // TODO: add inertia?

    // _d_sum = 0;
    // for (int i = 0; i < _d_n; i++)
    // {
    //     _d_sum += _d_buffer[i];
    // }
    _error_d = (error - _d_buffer[(_iter+1)%_d_n]) / (dt_ * (_d_n-1)) * Kd_;

    double error_sum = _error_p + _error_i + _error_d;

    _iter = (_iter + 1) % _d_n;
    return error_sum;
}
