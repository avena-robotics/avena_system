#include "candriver/candriver.h"

CanInterface::CanInterface(std::string can_id)
{

    std::cout << "Opening CAN socket..." << std::endl;

    int enable_canfd = 1;
    while ((_can_s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) == -1)
    {
        std::cout << "Encountered an error while opening CAN socket." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    while ((setsockopt(_can_s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd))))
    {
        std::cout << "Encountered an error while setting socket to CANFD ." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "Configuring IO..." << std::endl;

    try
    {
        // timeval tv;
        // tv.tv_sec = 0;
        // tv.tv_usec = 20;
        //not reliable
        // setsockopt(_can_s, SOL_SOCKET, SO_RCVTIMEO_NEW, &tv, sizeof(tv));
        int prio = 0;
        setsockopt(_can_s, SOL_SOCKET, SO_PRIORITY, &prio, sizeof(prio));
        strcpy(_ifr.ifr_name, can_id.c_str());
        ioctl(_can_s, SIOCGIFINDEX, &_ifr);
        fcntl(_can_s, F_SETFL, O_NONBLOCK);
        memset(&_can_addr, 0, sizeof(_can_addr));
        _can_addr.can_family = AF_CAN;
        _can_addr.can_ifindex = _ifr.ifr_ifindex;

        bind(_can_s, (sockaddr *)&_can_addr, sizeof(_can_addr));
        std::cout << "Successfully configured IO." << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
    catch (...)
    {
        std::cout << "Undefined exception occured" << std::endl;
    }
}

CanInterface::~CanInterface()
{
}

ResponseMsg CanInterface::getResponse(int expected_size)
{
    if (_response_msg.rx_msgs.size() >= expected_size)
    {
        return _response_msg;
    }
    else
    {
        std::vector<int16_t> single_rx_msg;
        std::chrono::time_point<std::chrono::steady_clock> _t_start = std::chrono::steady_clock::now();
        std::chrono::time_point<std::chrono::steady_clock> _t_last_msg = std::chrono::steady_clock::now();
        while ((_response_msg.rx_msgs.size() < expected_size) && (std::chrono::steady_clock::now() - _t_start < std::chrono::milliseconds(1)))
        {

            if (readFrame(0))
            {
                _t_last_msg = std::chrono::steady_clock::now();
                single_rx_msg.push_back(static_cast<int16_t>(_rx_frame.can_id));
                single_rx_msg.push_back(static_cast<int16_t>(_rx_frame.flags));
                for (int i = 0; i < _rx_frame.len; i++)
                {
                    single_rx_msg.push_back(static_cast<int16_t>(_rx_frame.data[i]));
                }
                // fprint_long_canframe(stdout, &_rx_frame, "\n", 0, CANFD_MAX_DLEN);
                // for (int i = 0; i < single_rx_msg.size(); i++)
                // {
                //     std::cout << single_rx_msg[i] << "/";
                // }

                _response_msg.rx_msgs.push_back(single_rx_msg);
                single_rx_msg.clear();

                // std::cout << std::endl;
                // std::cout << std::chrono::duration_cast<std::chrono::microseconds>(_t_last_msg - _t_start).count() << std::endl;
                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }
        }
        _response_msg.response_timestamp = std::chrono::steady_clock::now();
        // std::cout<<std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-_t_start).count()<<std::endl;
        return _response_msg;
    }
}

bool CanInterface::sendMessage(std::string msg, std::chrono::microseconds read_duration)
{
    std::chrono::time_point<std::chrono::steady_clock> _t_start = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> _t_last_msg = std::chrono::steady_clock::now();
    // std::cout << "Sending message " << msg << std::endl;
    char can_cstr[msg.size() + 1];
    std::vector<int16_t> single_rx_msg;

    _response_msg.rx_msgs.clear();
    strcpy(can_cstr, msg.c_str());
    // std::cout << "Message copied succesfully." << std::endl;
    parse_canframe(can_cstr, &_tx_frame);
    // std::cout << "Message parsed succesfully." << std::endl;

    sendFrame();

    while (std::chrono::steady_clock::now() - _t_start < read_duration)
    {

        if (readFrame(0))
        {
            _t_last_msg = std::chrono::steady_clock::now();
            single_rx_msg.push_back(static_cast<int16_t>(_rx_frame.can_id));
            single_rx_msg.push_back(static_cast<int16_t>(_rx_frame.flags));
            for (int i = 0; i < _rx_frame.len; i++)
            {
                single_rx_msg.push_back(static_cast<int16_t>(_rx_frame.data[i]));
            }
            // fprint_long_canframe(stdout, &_rx_frame, "\n", 0, CANFD_MAX_DLEN);
            // for (int i = 0; i < single_rx_msg.size(); i++)
            // {
            //     std::cout << single_rx_msg[i] << "/";
            // }

            _response_msg.rx_msgs.push_back(single_rx_msg);
            single_rx_msg.clear();

            // std::cout << std::endl;
            // std::cout << std::chrono::duration_cast<std::chrono::microseconds>(_t_last_msg - _t_start).count() << std::endl;
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    }
    _response_msg.response_timestamp = std::chrono::steady_clock::now();
    // std::cout<<std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-_t_start).count()<<std::endl;
    return 1;
}

bool CanInterface::readFrame(bool verbose)
{
    int nbytes = -1;
    try
    {
        // std::chrono::time_point<std::chrono::steady_clock> _t_start = std::chrono::steady_clock::now();
        nbytes = read(_can_s, &_rx_frame, sizeof(struct canfd_frame));
        // std::cout<<std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-_t_start).count()<<std::endl;
    }
    catch (const std::exception &e)
    {
        std::cout << e.what() << std::endl;
        return 0;
    }

    if (nbytes < 0)
    {
        if (verbose)
            std::cout << "Read frame error." << std::endl;
        return 0;
    }
    return 1;
}

bool CanInterface::sendFrame()
{
    int nbytes = -1;
    try
    {
        nbytes = write(_can_s, &_tx_frame, CANFD_MTU);
    }
    catch (const std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
    if (nbytes < CANFD_MTU)
    {
        if (nbytes < 0)
        {
            std::cout << "Write frame error." << std::endl;
            return 0;
        }
        std::cout << "Write incomplete frame." << std::endl;
        return 0;
    }

    return 1;
}
