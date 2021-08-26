#include "candriver/can_node.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <thread>
#include <mutex>

#include <fstream>

// std::ofstream pos_device("/home/user/2021/ROS_2/simple_controller/data/pos_device.txt");
// #define GEAR_RATIO 100
// bool checkUpdate()
// {
//     bool result = true;
//     if (device_dict["joint0"].last_update <= 0)
//     {
//         device_dict["joint0"].command_torque = 0;
//         result = false;
//     }
//     if (device_dict["joint1"].last_update <= 0)
//     {
//         device_dict["joint1"].command_torque = 0;
//         result = false;
//     }
//     return result;
// }

// void CanNode::subscribeCommand(const custom_interfaces::msg::JointCommand::SharedPtr msg)
// {
//     const std::lock_guard<std::mutex> lock(device_mutex_t); // check if working good

//     // devices[0].command_torque = msg->torque[0];
//     // devices[0].turn_motor = msg->turn_motor[0];

//     // devices[1].command_torque = msg->torque[1];
//     // devices[1].turn_motor = msg->turn_motor[1];
//     for (size_t jnt_idx; jnt_idx < msg->torque.size(); jnt_idx++)
//     {
//         devices[jnt_idx].command_torque = msg->torque[jnt_idx];
//         devices[jnt_idx].turn_motor = msg->turn_motor[jnt_idx];
//     }

//     // RCLCPP_INFO(this->get_logger(), "torque command 0 : %d", msg->torque[0]);
//     // RCLCPP_INFO(this->get_logger(), "torque command 1 : %d", msg->torque[1]);
// }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // set loop rate in ms
    // auto rate = std::chrono::milliseconds(INTERVAL);

    auto node = std::make_shared<CanNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "CAN driver node is closed.");

    return 0;
}

// void CanNode::loop()
// {

//     RCLCPP_INFO(this->get_logger(), "---------- STEP ----------");

//     // get calculated command torque
//     CAN_request_msg_t req_msg = getSrvMessage();
//     // printing (temporary here)
//     // RCLCPP_INFO(this->get_logger(), "joint 0 torque   : %7.4f", req_msg[0].torque);
//     // RCLCPP_INFO(this->get_logger(), "joint 1 torque   : %7.4f", req_msg[1].torque);

//     // RCLCPP_INFO(this->get_logger(), "torque command 0 : %d", devices[0].command_torque);
//     // RCLCPP_INFO(this->get_logger(), "torque command 1 : %d", devices[1].command_torque);

//     // RCLCPP_INFO(this->get_logger(), "joint 0 torque   : %d", req_msg[0].torque);
//     // RCLCPP_INFO(this->get_logger(), "joint 1 torque   : %d", req_msg[1].torque);
//     for (size_t jnt_idx = 0; jnt_idx < req_msg.size(); jnt_idx++)
//         RCLCPP_INFO_STREAM(this->get_logger(), "joint: " << jnt_idx << " has torque : " << req_msg[jnt_idx].torque);

//     // preapare frames to send
//     this->_can.createMessage(req_msg);

//     // send request and read response messages
//     CAN_response_msg_t res_msg;
//     // if (checkUpdate())
//     res_msg = this->_can.writeReadMessage();
//     // else
//     // res_msg = this->_can.writeEmergencyStop();

//     // RCLCPP_INFO(this->get_logger(), "joint 0 position : %7.4f", res_msg[0].position);
//     // RCLCPP_INFO(this->get_logger(), "joint 1 position : %7.4f", res_msg[1].position);

//     // create data for getter service
//     // this->setSrvMessage(res_msg);
//     this->publishStateMsg(res_msg);
// }

// void CanNode::getParams()
// {
//     // this will add parameters to device dict
// }

CAN_request_msg_t CanNode::getSrvMessage()
{
    auto t_current=std::chrono::steady_clock::now();
    // const std::lock_guard<std::mutex> lock(device_mutex_t_);
    // RCLCPP_INFO(this->get_logger(), "getSrvMessage is called");

    CAN_request_msg_t request;
    for (size_t jnt_idx = 0; jnt_idx < devices.size(); jnt_idx++)
    {
        request.at(jnt_idx).torque = devices.at(jnt_idx).command_torque;
        request.at(jnt_idx).turn_motor = devices.at(jnt_idx).turn_motor;
    }
    // request[0].torque = devices[0].command_torque;
    // request[0].turn_motor = devices[0].turn_motor;

    // request[1].torque = devices[1].command_torque;
    // request[1].turn_motor = devices[1].turn_motor;
    // std::cout<<"getSrvMessage "<<std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-t_current).count()<<std::endl;
    return request;
}

void CanNode::publishStateMsg(CAN_response_msg_t response)
{
        auto t_current=std::chrono::steady_clock::now();

    // auto message = custom_interfaces::msg::JointState();
    auto message = std::make_unique<sensor_msgs::msg::JointState>();
    // custom_interfaces::msg::JointState::UniquePtr message(new custom_interfaces::msg::JointState());

    for (size_t jnt_idx = 0; jnt_idx < response.size(); jnt_idx++)
    {
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("publish"), " i : "<< jnt_idx);
        // message->position.emplace_back(static_cast<double>(response[jnt_idx].position));
        // message->effort.emplace_back(static_cast<double>(response[jnt_idx].torque));

        message->position.emplace_back(static_cast<double>(response[jnt_idx].position * 2 * M_PI / GEAR_CONST / _can.gears_ratio.at(jnt_idx)));
        message->effort.emplace_back(static_cast<double>(response[jnt_idx].torque * TORQUE_CONSTANT * _can.gears_ratio.at(jnt_idx) * MOTOR_MAX_CURRENT / INT16_MAX));
        // message->effort.emplace_back(static_cast<double>(response[jnt_idx].torque));

        // message->status = {response[0].motor_status, response[1].motor_status};
    }

    // message.position[1] = response[1].position;
    // message.torque[1] = response[1].torque;
    // message.status[1] = response[1].motor_status;
    message->header.stamp = rclcpp::Clock().now();
    this->state_publisher_->publish(std::move(message));
    // this->state_publisher_->publish(message);
    // std::cout<<"publishStateMsg "<<std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-t_current).count()<<std::endl;
}

// void CanNode::setSrvMessage(CAN_response_msg_t response)
// {
//     const std::lock_guard<std::mutex> lock(device_mutex_p);

//     devices[0].read_position = response[0].position;
//     devices[0].read_torque = response[0].torque;
//     devices[0].read_status = response[0].motor_status;

//     devices[1].read_position = response[1].position;
//     devices[1].read_torque = response[1].torque;
//     devices[1].read_status = response[1].motor_status;

//     pos_device << devices[1].read_position << "\n";
//     pos_device.flush();
// }