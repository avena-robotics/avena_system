#ifndef CANDRIVER_H_
#define CANDRIVER_H_

#include <linux/can/raw.h>
#include <linux/can.h>

extern "C"
{
#include "candriver/lib.h"
}

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <unistd.h>

#include <functional>
#include <cstdlib>
#include <vector>
#include <memory>
#include <iostream>
#include <chrono>
#include <thread>
#include <string.h>
#include <cstring>

#include <fcntl.h>

struct ResponseMsg
{
    std::chrono::steady_clock::time_point response_timestamp; //in microseconds
    std::vector<std::vector<int16_t>> rx_msgs;
};

class CanInterface
{
public:
    CanInterface(std::string can_id = "can1");
    ~CanInterface();

    ResponseMsg getResponse(size_t expected_msg = 0);
    bool sendMessage(std::string msg, std::chrono::microseconds read_duration);

private:
    int _can_s;
    sockaddr_can _can_addr;
    ifreq _ifr;
    canfd_frame _rx_frame, _tx_frame;

    ResponseMsg _response_msg;
    bool sendFrame();
    bool readFrame(bool verbose);
};
#endif // CANDRIVER_H_