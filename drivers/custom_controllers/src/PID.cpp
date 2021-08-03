#include <custom_controllers/PID.hpp>


    PID::PID(double Kp, double Ki,double Kd, double dt){
        Kp_=Kp;
        Ki_=Ki;
        Kd_=Kd;
        dt_=dt;
        i_val_=0;
        prev_error_=0;
        i_clamp_high_=DBL_MAX;
        i_clamp_low_=-DBL_MAX;
    }

        PID::PID(double Kp, double Ki,double Kd, double dt, double i_clamp_low, double i_clamp_high){
        Kp_=Kp;
        Ki_=Ki;
        Kd_=Kd;
        dt_=dt;
        i_clamp_low_=i_clamp_low;
        i_clamp_high_=i_clamp_high;
        i_val_=0;
        prev_error_=0;
    }

    PID::~PID(){}

    void PID::update(double Kp, double Ki,double Kd){
        Kp_=Kp;
        Ki_=Ki;
        Kd_=Kd;
    }

    double PID::getValue(double error){


        double error_p=error*Kp_;
        i_val_+=(error*dt_);

        if (i_val_>i_clamp_high_){
            i_val_=i_clamp_high_;
        }
        if (i_val_<i_clamp_low_){
            i_val_=i_clamp_low_;
        }

        double error_i=i_val_*Ki_;

        double error_d=(error-prev_error_)/dt_*Kd_;
        prev_error_=error;


        double error_sum=error_p+error_i+error_d;
        return error_sum; 
    }

