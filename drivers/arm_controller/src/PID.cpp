#include <arm_controller/PID.hpp>


    PID::PID(double Kp, double Ki,double Kd, double dt, int d_n){
        Kp_=Kp;
        Ki_=Ki;
        Kd_=Kd;
        dt_=dt;
        i_val_=0;
        prev_error_=0;
        i_clamp_high_=DBL_MAX;
        i_clamp_low_=-DBL_MAX;
        _d_n=d_n;
        _d_buffer.resize(d_n);
        _iter=0;
    }

        PID::PID(double Kp, double Ki,double Kd, double dt, double i_clamp_low, double i_clamp_high, int d_n){
        Kp_=Kp;
        Ki_=Ki;
        Kd_=Kd;
        dt_=dt;
        i_clamp_low_=i_clamp_low/Ki;
        i_clamp_high_=i_clamp_high/Ki;
        i_val_=0;
        prev_error_=0;
        _d_n=d_n;
        _d_buffer.resize(d_n);
        _iter=0;
    }

    PID::~PID(){}

    void PID::update(double Kp, double Ki,double Kd){
        Kp_=Kp;
        Ki_=Ki;
        Kd_=Kd;
    }

    double PID::getValue(double error){

        _d_buffer[_iter]=error;

        double error_p=error*Kp_;
        i_val_+=(error*dt_);

        if (i_val_>i_clamp_high_){
            i_val_=i_clamp_high_;
        }
        if (i_val_<i_clamp_low_){
            i_val_=i_clamp_low_;
        }

        double error_i=i_val_*Ki_;

        //TODO: add inertia?

        _d_sum=0;
        for(int i=0;i<_d_n;i++){
            _d_sum+=_d_buffer[i];
        }
        double error_d=(_d_sum-prev_error_)/(dt_*_d_n)*Kd_;
        prev_error_=_d_sum;


        double error_sum=error_p+error_i+error_d;

        _iter=(_iter+1)%_d_n;
        return error_sum; 
    }

