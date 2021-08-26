#pragma once

#define JOINTS_NUMBER 6
#define DELTA_T 0.004    //unused
#define INTERVAL 20     //unused

#define TRESHOLD 0.05
#define TRESHOLD_2 0.01

#define ERROR_TH0 0.005
#define ERROR_TH1 0.005


#define GEAR_RATIO_1 120.
#define GEAR_RATIO_2 120.
#define GEAR_RATIO_3 120.
#define GEAR_RATIO_4 120.
#define GEAR_RATIO_5 120.
#define GEAR_RATIO_6 120.
#define GEAR_CONST 84   //?

#define TORQUE_CONSTANT 0.1118
#define MOTOR_MAX_CURRENT 31.853    //?

#define CAN_NUMBER "can1"


#define MIN_POSITION_1      -1.57
#define MAX_POSITION_1       1.57
#define MAX_VELOCITY_1       2
#define MAX_ACCELERATION_1   5
#define MAX_JERK_1           10
#define MAX_TOURQUE_1        120
#define MAX_ROTATUM_1        1000


#define MIN_POSITION_2      -1.57
#define MAX_POSITION_2       1.57
#define MAX_VELOCITY_2       2
#define MAX_ACCELERATION_2   5
#define MAX_JERK_2           10
#define MAX_TOURQUE_2        50
#define MAX_ROTATUM_2        1000

#define CHART_DRAW 0

#define FILTER_SIZE 2
#define FILTER_SIZE_INIT 2

#define FILTER_SIZE_1 1
#define FILTER_SIZE_2 1