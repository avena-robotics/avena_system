#pragma once

#include <string>
#include <map>
#include <chrono>
#include <queue>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "candriver/can_operations.h"

// #include "custom_interfaces/srv/comm_values.hpp"
// #include "custom_interfaces/srv/get_value.hpp"
// #include "custom_interfaces/srv/set_value.hpp"

#include "custom_interfaces/srv/set_arm_torques.hpp"
#include "custom_interfaces/srv/get_arm_state.hpp"

// #include "custom_interfaces/msg/joint_state.hpp"
// #include "custom_interfaces/msg/joint_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "std_msgs/msg/int16.hpp"
#include "helpers_commons/helpers_commons.hpp"
#include "types.h"

// write a class of can node here?

enum InternalState
{
    START = 0x01,
    NOT_READY_TO_SWITCH_ON = 0x02,
    SWITCH_ON_DISABLED = 0x03,
    READY_TO_SWITCH_ON = 0x04,
    SWITCHED_ON = 0x05,
    OPERATION_ENABLE = 0x06,
    QUICK_STOP_ACTIVE = 0x07,
    FAULT_REACTION_ACTIVE = 0x08,
    FAULT = 0x09,
    UNDEFINED = 0xFF
};
enum ErrorCode : uint8_t
{
    SUCCESS = EXIT_SUCCESS,
    ACTIVE_ERROR_ROS = EXIT_FAILURE,
    PAST_ERROR_ROS = 255
};

struct sensor_int32_status
{
    rclcpp::Time update_time; // do not know if it works
    int32_t value;            // external sensor
};

struct atomic_device_status
{
    // values that are responsed
    int32_t read_position;
    int32_t read_torque;
    int32_t read_status;

    // values that are requested
    int32_t command_torque;
    int32_t turn_motor;

    // updating info data
    // 5,4,3,2,1 - CAN will send message; 0 - launch emergency stop
    // int32_t last_update = 0;

    // data
    int gear_ratio;
    int pole_pairs;
};

// struct device_status
// {
//     // values that are responsed
//     float read_position;
//     float read_torque;
//     int32_t read_status;

//     std::atomic<int32_t> read_position2;

//     // values that are requested
//     float command_torque;
//     int32_t turn_motor;

//     // updating info data
//     // 5,4,3,2,1 - CAN will send message; 0 - launch emergency stop
//     int32_t last_update = 0;

//     // data
//     int gear_ratio;
//     int pole_pairs;
// };

// can_transmition::CanTransmition *can_;
atomic_device_status joint_0;
atomic_device_status joint_1;

int8_t hal_states = 6;
// std::map<std::string, atomic_device_status> device_dict; // joint name and joint status struct

joints<atomic_device_status> devices;
std::map<int, std::string> can_id_dict;

std::queue<int32_t> *position_1_buffer;
std::queue<int32_t> *position_2_buffer;

// functions
void get_params();
// bool get_position_service(candriver::srv::GetValue::Request &req, candriver::srv::GetValue::Response &res);

class CanNode : public rclcpp::Node, public helpers::WatchdogInterface
{
public:
    void initNode() override
    {
        status = custom_interfaces::msg::Heartbeat::RUNNING;
    };
    void shutDownNode() override
    {
        status = custom_interfaces::msg::Heartbeat::STOPPED;
        return;
    };
    helpers::Watchdog::SharedPtr _watchdog;
    CanNode() : Node("can_node")
    {

        _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
        status = custom_interfaces::msg::Heartbeat::STOPPED;
        _can.initSocket();
        // _can.readOnlyPosition();
        auto send_torque_service = [this](const std::shared_ptr<custom_interfaces::srv::SetArmTorques::Request> request,
                                          std::shared_ptr<custom_interfaces::srv::SetArmTorques::Response> response) -> void
        {
            auto t_current = std::chrono::steady_clock::now();
            // RCLCPP_INFO(this->get_logger(), "Set service is called");

            // RCLCPP_INFO(this->get_logger(),"request torque: ",devices.at(0).turn_motor);
            // RCLCPP_INFO(this->get_logger(),"request torque: ",request->torques.at(0));
            // RCLCPP_INFO(this->get_logger(),"request motor turn: ",request->turn_motor.at(0));
            // RCLCPP_INFO(this->get_logger(),"device read status: ",devices.at(0).read_status);
            for (size_t jnt_idx = 0; jnt_idx < JOINTS_NUMBER; jnt_idx++)
            {
                // RCLCPP_INFO(this->get_logger(), "dupa1 %i",std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-t_current).count());
                // const std::lock_guard<std::mutex> lock(device_mutex_t_); // check if working good
                // RCLCPP_INFO(this->get_logger(), "dupa2 %i",std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-t_current).count());
                devices.at(jnt_idx).command_torque = static_cast<int32_t>(request->torques.at(jnt_idx) * double(INT16_MAX) / MOTOR_MAX_CURRENT / TORQUE_CONSTANT / double(_can.gears_ratio.at(jnt_idx)));
                // RCLCPP_INFO(this->get_logger(), "dupa3 %i",std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-t_current).count());
                devices.at(jnt_idx).turn_motor = request->turn_motor.at(jnt_idx);
                // RCLCPP_INFO(this->get_logger(), "dupa4 %i",std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-t_current).count());
                // response->error.at(jnt_idx) = devices.at(jnt_idx).read_status;
            }
            // RCLCPP_INFO_STREAM(this->get_logger(), "tq: "<<request->torques.at(0));
            // RCLCPP_INFO(this->get_logger(),"after filling");
            // torque_1_buffer->push(request->torque);

            // set update value
            // device_dict[request->device_name].last_update = 5;

            // loop();

            // get calculated command torque
            CAN_request_msg_t req_msg = getSrvMessage();
            // preapare frames to send
            // _can.createMessage(req_msg);
            _can.createBroadcastMessage(req_msg);

            // send request and read response messages
            CAN_response_msg_t res_msg;
            // res_msg = _can.writeReadMessage();
            res_msg = _can.writeReadFdMessage();
            response->error.resize(JOINTS_NUMBER);
            for (size_t jnt_idx = 0; jnt_idx < JOINTS_NUMBER; jnt_idx++)
            {
                if (res_msg.at(jnt_idx).error_code != 0)
                {
                    if (res_msg.at(jnt_idx).joint_status == JointErrorCode::PAST_ERROR)
                    {
                        response->error.at(jnt_idx) = ErrorCode::PAST_ERROR_ROS;
                    }
                    else if (res_msg.at(jnt_idx).joint_status == JointErrorCode::ACTIVE_ERROR)
                    {
                        response->error.at(jnt_idx) = ErrorCode::ACTIVE_ERROR_ROS;
                    }
                }
                else
                    response->error.at(jnt_idx) = ErrorCode::SUCCESS;
            }
            publishStateMsg(res_msg);
            // RCLCPP_INFO(this->get_logger(), "Set %i",std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-t_current).count());
            // RCLCPP_INFO(this->get_logger(), "sent set response: %i", std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-t_current).count());
        };

        auto read_status_service = [this](const std::shared_ptr<custom_interfaces::srv::GetArmState::Request> request,
                                          std::shared_ptr<custom_interfaces::srv::GetArmState::Response> response) -> void
        {
            auto t_current = std::chrono::steady_clock::now();
            // RCLCPP_INFO(this->get_logger(), "Get service is called");
            // auto t_current = std::chrono::steady_clock::now();
            // const std::lock_guard<std::mutex> lock(device_mutex_p_); // check if working good
            (void)request;
            // response->position = devices[request->device_name].read_position;
            // response->torque = devices[request->device_name].read_torque;
            // response->status = devices[request->device_name].read_status;
            response->arm_current_positions.resize(JOINTS_NUMBER);
            response->arm_current_torques.resize(JOINTS_NUMBER);
            response->arm_current_status.resize(JOINTS_NUMBER);
            response->arm_current_temperature.resize(JOINTS_NUMBER);

            // if(request->device_name == 0 && !position_1_buffer->empty())
            // {
            //     response->position = position_1_buffer->front();
            //     position_1_buffer->pop();
            // }

            // if(request->device_name == 1 && !position_2_buffer->empty())
            // {
            //     response->position = position_2_buffer->front();
            //     position_2_buffer->pop();
            // }

            // response->torque = torque_1_buffer->front();
            // loop();
            CAN_request_msg_t req_msg = getSrvMessage();

            // preapare frames to send
            // _can.createMessage(req_msg);
            _can.createBroadcastMessage(req_msg);

            // send request and read response messages
            CAN_response_msg_t res_msg;
            // res_msg = _can.writeReadMessage();
            res_msg = _can.writeReadFdMessage();

            for (size_t jnt_idx = 0; jnt_idx < devices.size(); jnt_idx++)
            {
                std::cout << "int val: " << res_msg.at(jnt_idx).position << " X " << (2 * M_PI / GEAR_CONST / double(_can.gears_ratio.at(jnt_idx))) << std::endl;
                response->arm_current_positions.at(jnt_idx) = double(res_msg.at(jnt_idx).position) * (2 * M_PI / GEAR_CONST / double(_can.gears_ratio.at(jnt_idx)));
                response->arm_current_torques.at(jnt_idx) = res_msg.at(jnt_idx).torque * TORQUE_CONSTANT * _can.gears_ratio.at(jnt_idx) * MOTOR_MAX_CURRENT / INT16_MAX;
                response->arm_current_temperature.at(jnt_idx) = res_msg.at(jnt_idx).temperature;
                response->arm_current_status.at(jnt_idx) = res_msg.at(jnt_idx).joint_status;
            }
            publishStateMsg(res_msg);
            // RCLCPP_INFO(this->get_logger(), "Get %i",std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-t_current).count());
            // RCLCPP_INFO(this->get_logger(), "sent get response: %i", std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-t_current).count());
        };

        // command_subscription_ = this->create_subscription<custom_interfaces::msg::JointCommand>(
        //     "Joint_commands", 10, std::bind(&CanNode::subscribeCommand, this, std::placeholders::_1));

        state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm_joint_states", 10);
        send_service_ = this->create_service<custom_interfaces::srv::SetArmTorques>("/arm_controller/set_torques", send_torque_service);
        read_service_ = this->create_service<custom_interfaces::srv::GetArmState>("/arm_controller/get_current_arm_state", read_status_service);
        // timer_ = this->create_wall_timer(loop_rate, std::bind(&CanNode::loop, this));
    }

    

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
    rclcpp::Service<custom_interfaces::srv::SetArmTorques>::SharedPtr send_service_;
    rclcpp::Service<custom_interfaces::srv::GetArmState>::SharedPtr read_service_;

    // rclcpp::Subscription<custom_interfaces::msg::JointCommand>::SharedPtr command_subscription_;

    // rclcpp::TimerBase::SharedPtr timer_;

    // void getParams();
    // void loop();

    void setSrvMessage(CAN_response_msg_t response);
    void publishStateMsg(CAN_response_msg_t response);
    CAN_request_msg_t getSrvMessage();

    // void subscribeCommand(const custom_interfaces::msg::JointCommand::SharedPtr msg);

    CanOperations _can;
    std::mutex device_mutex_t_;
    std::mutex device_mutex_p_;
};
