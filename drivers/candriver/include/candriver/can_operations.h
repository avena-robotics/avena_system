#pragma once

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include <typeinfo>
#include <cstdint>
#include <iostream>
#include <chrono>
#include <thread>
#include <functional>
#include <ctime>
#include <mutex>
#include <cmath>
#include <condition_variable>

// CAN
#include <linux/can.h>
#include <linux/can/raw.h>

// some more CAN
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "candriver/types.h"
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
enum JointErrorCode : uint8_t
{
    PAST_ERROR = 10,
    ACTIVE_ERROR = 11,
};
class CanOperations
{

public:
    bool initSocket();
    // bool createMessage(CAN_request_msg_t message);
    bool createBroadcastMessage(CAN_request_msg_t message);
    // void SendAck();
    // CAN_response_msg_t readOnlyPosition();
    // CAN_response_msg_t writeReadMessage();
    CAN_response_msg_t writeReadFdMessage();
    // CAN_response_msg_t writeEmergencyStop();

    bool restart(int joint_number);

    // joints_d gears_ratio{GEAR_RATIO_1};
    joints_d gears_ratio{GEAR_RATIO_1, GEAR_RATIO_2, GEAR_RATIO_3, GEAR_RATIO_4, GEAR_RATIO_5};

    int sock;
    struct sockaddr_can addr;
    struct ifreq ifr;
    // joints<can_frame> frames;
    joints<canfd_frame> frames;
    canfd_frame temp_frame;
    joints<int16_t> last_postion_msg_value = {0};
    // joints<int16_t> last_postion_msg_value = {0,0};
    joints<int16_t> last_torque_msg_value = {0};
    // joints<int16_t> last_torque_msg_value = {0,0};
    joints<int8_t> last_motor_msg_value = {1};
    // joints<int8_t> last_motor_msg_value = {1,2};

    int getFrameId(int joint_number);
    // CAN_single_response_msg_t readMessage(const int joint_number);
    CAN_single_response_msg_t readJointMessage(int joint);

private:
    // old
    std::chrono::steady_clock::time_point time_point;
    bool error_appear;
    struct can_frame frame;
};