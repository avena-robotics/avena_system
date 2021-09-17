#include "candriver/candriver.h"
#include <math.h>
#include <thread>
#include <mutex>

struct JointStatus
{
    double position, velocity, torque, temperature;
    int state, current_error, prev_error;
};

struct JointCommand
{
    double c_torque;
    int c_status;
};

struct ArmStatus
{
    std::vector<JointStatus> joints;
    std::chrono::steady_clock::time_point timestamp;
};

struct ArmCommand
{
    std::vector<JointCommand> joints;
    std::chrono::steady_clock::time_point timestamp;
};

class ArmInterface
{

public:
    ArmInterface(std::string can_addr, int fq)
    {
        _can_addr = can_addr;
        std::cout << "Initializing connection with arm." << std::endl;

        sendArmCommand(_arm_command, std::chrono::seconds(1));

        _arm_status.joints.resize(_last_msg.rx_msgs.size());
        _arm_command.joints.resize(_last_msg.rx_msgs.size());

        _get_timestamp = std::chrono::steady_clock::now();
        _command_timestamp = std::chrono::steady_clock::now();
        _status_timestamp = std::chrono::steady_clock::now();

        _max_delay = std::chrono::microseconds((int)(1000000. / fq) * 5);

        _joints_number = _last_msg.rx_msgs.size();

        std::cout << "Arm connection initialization successfull." << std::endl;

        std::thread comms_thread(&ArmInterface::startCommsLoop, this, fq);
        comms_thread.detach();
    }

    ~ArmInterface()
    {
    }

    ArmStatus getArmState()
    {
        _get_timestamp = std::chrono::steady_clock::now();
        _get_delay = std::chrono::duration_cast<std::chrono::microseconds>(_get_timestamp - _status_timestamp);

        return _arm_status;
    }

    bool setArmCommand(ArmCommand arm_command)
    {
        _arm_command = arm_command;
        _command_timestamp = arm_command.timestamp;
        return 1;
    }

private:
    CanInterface _can_interface;
    std::string _can_addr;
    ArmStatus _arm_status;
    ArmCommand _arm_command;
    ResponseMsg _last_msg;
    int _joints_number;

    std::mutex _arm_command_mutex, _arm_status_mutex;

    std::chrono::steady_clock::time_point _status_timestamp, _command_timestamp, _get_timestamp;
    std::chrono::microseconds _max_delay, _get_delay;

    //PHYSICAL PARAMS
    double _gear_ratio = 120.;
    double _gear_const = 84;
    double _torque_const = 0.1118;
    double _motor_max_current = 31.853;

    double _torque_multiplier = _torque_const * _gear_ratio * _motor_max_current / INT16_MAX;
    double _position_multiplier = 2 * M_PI / _gear_const / _gear_ratio;

    void startCommsLoop(int fq)
    {
        std::chrono::microseconds ref_read_time((int)(1000000. / fq));
        std::chrono::microseconds read_time = ref_read_time;

        while (1)
        {
            // if (_get_delay > std::chrono::microseconds(100))
            // {
            //     if (_get_delay > std::chrono::microseconds(1100))
            //     {
            //         read_time = _get_delay - std::chrono::microseconds(100);
            //     }
            //     else
            //     {
            //         std::this_thread::sleep_for(_get_delay - std::chrono::microseconds(100));
            //     }
            // }

            // if (read_time < ref_read_time/2)
            // {
            //     read_time = ref_read_time;
            // }

            if (std::chrono::steady_clock::now() - _command_timestamp > _max_delay)
            {
                sendArmCommand(ArmCommand(), read_time);
            }
            else
            {
                sendArmCommand(_arm_command, read_time);
            }

            updateArmState();
        }
    }

    bool sendArmCommand(ArmCommand arm_command, std::chrono::microseconds read_time)
    {
        std::stringstream can_msg_str;
        std::scoped_lock(_arm_status_mutex);

        can_msg_str << _can_addr << "##1";
        for (size_t i = 0; i < arm_command.joints.size(); i++)
        {
            can_msg_str << std::hex << std::setfill('0') << std::setw(4) << (int16_t)(arm_command.joints[i].c_torque / _torque_multiplier);
            can_msg_str << std::hex << std::setfill('0') << std::setw(2) << (int16_t)(arm_command.joints[i].c_status);
        }
        // for (size_t i = 0; i < (6 - arm_command.joints.size()); i++)
        // {
        //     can_msg_str << "0000";
        // }
        // can_msg_str << "00";
        // for (size_t i = 0; i < arm_command.joints.size(); i++)
        // {
        //     can_msg_str << std::hex << (int)(arm_command.joints[i].c_status);
        // }
        // for (size_t i = 0; i < (6 - arm_command.joints.size()); i++)
        // {
        //     can_msg_str << "0";
        // }

        _can_interface.sendMessage(can_msg_str.str(), read_time);

        _last_msg = _can_interface.getResponse(_joints_number);
        _status_timestamp = _last_msg.response_timestamp;

        return 1;
    }

    bool updateArmState()
    {
        std::scoped_lock(_arm_status_mutex);
        int arm_id;

        for (size_t i = 0; i < _last_msg.rx_msgs.size(); i++)

        {
            arm_id = _last_msg.rx_msgs[i][0] / 16 - 10;
            _arm_status.joints[arm_id].position = (double)(static_cast<int16_t>(_last_msg.rx_msgs[i][2] << 8) ^ (_last_msg.rx_msgs[i][3])) * _position_multiplier;
            _arm_status.joints[arm_id].velocity = (double)(static_cast<int16_t>(_last_msg.rx_msgs[i][4] << 8) ^ (_last_msg.rx_msgs[i][5])) *  INT16_MAX;
            _arm_status.joints[arm_id].torque = (double)(static_cast<int16_t>(_last_msg.rx_msgs[i][6] << 8) ^ (_last_msg.rx_msgs[i][7])) * _torque_multiplier;
            _arm_status.joints[arm_id].temperature = _last_msg.rx_msgs[i][8];
            _arm_status.joints[arm_id].state = _last_msg.rx_msgs[i][9];
            _arm_status.joints[arm_id].current_error = _last_msg.rx_msgs[i][10];
            _arm_status.joints[arm_id].prev_error = _last_msg.rx_msgs[i][11];
        }

        _arm_status.timestamp = _status_timestamp;

        return 1;
    }
};