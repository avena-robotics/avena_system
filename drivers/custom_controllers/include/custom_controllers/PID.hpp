#pragma once
#include <float.h>

class PID
{
    public:


        PID(double Kp, double Ki,double Kd, double dt);
        PID(double Kp, double Ki,double Kd, double dt, double i_clamp_low, double i_clamp_high);
        ~PID();
        double getValue(double error);
        void update(double Kp, double Ki,double Kd);

    private:
        double Kp_, Ki_, Kd_, dt_, i_clamp_low_, i_clamp_high_;
        double i_val_, prev_error_;
        
        


};