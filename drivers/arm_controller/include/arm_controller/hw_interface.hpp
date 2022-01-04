#include "candriver/candriver.h"

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <cstdlib>
#include <math.h>
#include <thread>
#include <mutex>
#include <iostream>
#include <iomanip>

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

struct GripperStatus
{
    boost::interprocess::interprocess_mutex mutex;
    double position, val1, val2;
    int state, current_error, prev_error;
};

struct GripperCommand
{
    boost::interprocess::interprocess_mutex mutex;
    double c_val;
    int c_status;
};

struct ArmStatus
{
    boost::interprocess::interprocess_mutex mutex;
    JointStatus joints[6];
    std::chrono::steady_clock::time_point timestamp;
};

struct ArmCommand
{
    boost::interprocess::interprocess_mutex mutex;
    JointCommand joints[6];
    std::chrono::steady_clock::time_point timestamp;
};

class ArmInterface : public rclcpp::Node
{

public:
    ArmInterface(std::string can_addr, int fq) : Node("candriver")
    {
        std::cout << "Clear shared memory location" << std::endl;
        boost::interprocess::shared_memory_object::remove("HWSharedMemory");

        size_t var_mem_size = 0;
        var_mem_size = sizeof(ArmCommand) + sizeof(ArmStatus) + sizeof(GripperCommand) + sizeof(GripperStatus);

        std::cout << "Allocating " << var_mem_size << " bytes" << std::endl;

        std::cout << "Create shared memory location" << std::endl;
        _shared_memory_segment = std::make_shared<boost::interprocess::managed_shared_memory>(boost::interprocess::create_only, "HWSharedMemory", var_mem_size*2);
        

        std::cout << "Create shared memory variables" << std::endl;
        _arm_command = std::shared_ptr<ArmCommand>(_shared_memory_segment->construct<ArmCommand>("shmArmCommand")());
        _arm_status = std::shared_ptr<ArmStatus>(_shared_memory_segment->construct<ArmStatus>("shmArmStatus")());
        _gripper_command = std::shared_ptr<GripperCommand>(_shared_memory_segment->construct<GripperCommand>("shmGripperCommand")());
        _gripper_status = std::shared_ptr<GripperStatus>(_shared_memory_segment->construct<GripperStatus>("shmGripperStatus")());
        std::cout << "Done initializing shared memory" << std::endl;
        std::cout << _shared_memory_segment->get_free_memory()<< std::endl;
        _can_addr = can_addr;



        //init vars
        for (size_t i = 0; i < 6; i++)
        {
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> ac_lock(_arm_command->mutex);
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> as_lock(_arm_status->mutex);
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> gc_lock(_gripper_command->mutex);
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> gs_lock(_gripper_status->mutex);

            _arm_command->joints[i].c_status = 0;
            _arm_command->joints[i].c_torque = 0;

            _arm_status->joints[i].current_error = 21;
            _arm_status->joints[i].position = 0.;
            _arm_status->joints[i].prev_error = 37;
            _arm_status->joints[i].state = 420;
            _arm_status->joints[i].temperature = 0.;
            _arm_status->joints[i].torque = 0.;
            _arm_status->joints[i].velocity = 0.;

            _gripper_command->c_status = 0;
            _gripper_command->c_val = 0.;

            _gripper_status->current_error = 21;
            _gripper_status->position = 0.;
            _gripper_status->val1 = 0.;
            _gripper_status->val2 = 0.;
            _gripper_status->prev_error = 37;
            _gripper_status->state = 420;
        }


        while (_last_msg.rx_msgs.size() < 1)
        {
            std::cout << "Initializing connection with CAN." << std::endl;
            sendArmCommand(_arm_command, std::chrono::seconds(1));
            if (_last_msg.rx_msgs.size() < 1)
            {
                std::cout << "Cannot establish connection with CAN." << std::endl;
            }
            else
            {
                std::cout << "Established connection with CAN" << std::endl;
            }
        }


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
        boost::interprocess::shared_memory_object::remove("HWSharedMemory");
        std::cout << "Shared memory cleared" << std::endl;
    }

    // ArmStatus getArmState()
    // {
    //     _get_timestamp = std::chrono::steady_clock::now();
    //     _get_delay = std::chrono::duration_cast<std::chrono::microseconds>(_get_timestamp - _status_timestamp);

    //     return _arm_status;
    // }

    // bool setArmCommand(ArmCommand arm_command)
    // {
    //     *_arm_command = arm_command;
    //     _command_timestamp = arm_command.timestamp;
    //     return 1;
    // }

private:
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

            // if (std::chrono::steady_clock::now() - _command_timestamp > _max_delay)
            // {
            //     sendArmCommand(ArmCommand(), read_time);
            // }
            // else
            // {
            //     sendArmCommand(_arm_command, read_time);
            // }

            sendArmCommand(_arm_command, read_time);

            updateArmState();
        }
    }

    bool sendArmCommand(std::shared_ptr<ArmCommand> arm_command, std::chrono::microseconds read_time)
    {
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(_arm_command->mutex);
        std::stringstream can_msg_str;

        can_msg_str << _can_addr << "##1";
        for (size_t i = 0; i < 6; i++)
        {
            can_msg_str << std::hex << std::setfill('0') << std::setw(4) << (int16_t)(arm_command->joints[i].c_torque / _torque_multiplier);
            can_msg_str << std::hex << std::setfill('0') << std::setw(2) << (int16_t)(arm_command->joints[i].c_status);
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
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(_arm_status->mutex);
        int arm_id;

        for (size_t i = 0; i < _last_msg.rx_msgs.size(); i++)

        {
            arm_id = _last_msg.rx_msgs[i][0] / 16 - 10;
            _arm_status->joints[arm_id].position = (double)(static_cast<int16_t>(_last_msg.rx_msgs[i][2] << 8) ^ (_last_msg.rx_msgs[i][3])) * _position_multiplier;
            _arm_status->joints[arm_id].velocity = (double)(static_cast<int16_t>(_last_msg.rx_msgs[i][4] << 8) ^ (_last_msg.rx_msgs[i][5])) / INT16_MAX * 2 * M_PI;
            _arm_status->joints[arm_id].torque = (double)(static_cast<int16_t>(_last_msg.rx_msgs[i][6] << 8) ^ (_last_msg.rx_msgs[i][7])) * _torque_multiplier;
            _arm_status->joints[arm_id].temperature = _last_msg.rx_msgs[i][8];
            _arm_status->joints[arm_id].state = _last_msg.rx_msgs[i][9];
            _arm_status->joints[arm_id].current_error = _last_msg.rx_msgs[i][10];
            _arm_status->joints[arm_id].prev_error = _last_msg.rx_msgs[i][11];
        }

        _arm_status->timestamp = _status_timestamp;

        return 1;
    }

    CanInterface _can_interface;
    std::string _can_addr;
    std::shared_ptr<ArmStatus> _arm_status;
    std::shared_ptr<ArmCommand> _arm_command;
    std::shared_ptr<GripperCommand> _gripper_command;
    std::shared_ptr<GripperStatus> _gripper_status;
    ResponseMsg _last_msg;
    std::shared_ptr<boost::interprocess::managed_shared_memory> _shared_memory_segment;
    int _joints_number;

    std::chrono::steady_clock::time_point _status_timestamp, _command_timestamp, _get_timestamp;
    std::chrono::microseconds _max_delay, _get_delay;

    //PHYSICAL PARAMS
    double _gear_ratio = 121.;
    double _gear_const = 84;
    double _torque_const = 0.1118;
    double _motor_max_current = 31.853;

    // double _torque_multiplier = _torque_const * _gear_ratio * _motor_max_current / (double)INT16_MAX;
    double _torque_multiplier = 256. / (double)INT16_MAX;
    // double _position_multiplier = 2 * M_PI / _gear_const / _gear_ratio;
    double _position_multiplier = M_PI / (double)INT16_MAX;
};