#pragma once

#include <array>
#include <string>
#include "constants.h"

template <typename T>
using joints = std::array<T, JOINTS_NUMBER>;

typedef joints<double> joints_d;
typedef joints<int> joints_i;
// typedef joints<std::string> joints_s;

enum JOB_STATUS
{
    HOLD,
    LOAD_TRAJ,
    MOVE,
    STOP,
    POWEROFF
};

enum ENABLE_JOINT
{
    DISABLED,
    ENABLED  

};

struct PID_values_t
{
    double P;
    double I;
    double D;
};

struct point_t
{
    double position;
    double velocity;
    double acceleration;
};

struct CAN_single_response_msg_t
{
    int joint_id;
    int16_t position;
    int16_t torque;
    int16_t joint_status;
    int16_t error_code;
};

struct CAN_single_request_msg_t
{
    int16_t torque;
    int8_t turn_motor;
};

struct Safety_Message_t
{
    std::string message;
    bool is_safe;

    Safety_Message_t(std::string message, bool value) : message(message), is_safe(value){};
};

struct ParametersStruct
{
    double breakTau;
    double coulumbTau;
    double strikeV;
    double brakePercent;
    double afterJobP;
    double afterJobI;
    double afterJobD;
    double accelFF;
    double beta;
};


typedef joints<PID_values_t> joints_PID_array;
typedef joints<ENABLE_JOINT> enable_joint_t;
typedef joints<point_t> goal_point_t;
typedef joints<CAN_single_response_msg_t> CAN_response_msg_t;
typedef joints<CAN_single_request_msg_t> CAN_request_msg_t;
typedef joints<ParametersStruct> parameters;



inline int signum(double x)
{
    return x >= 0 ? 1 : -1;
}

inline int signum3(double x)
{
    if (x > 0)
        return 1;
    else if (x < 0)
        return -1;
    return 0;
}