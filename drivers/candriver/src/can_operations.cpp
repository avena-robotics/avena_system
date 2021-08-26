// sudo ip link set canx up type can 1000000 triple-sampling fd on

// sudo ifconfig canX up
#include "candriver/can_operations.h"

#include <fstream>

// std::ofstream pos_can("/home/user/2021/ROS_2/simple_controller/data/pos_can.txt");

//
//  get frame id in terms of joint number
//
int CanOperations::getFrameId(int joint_number)
{
    return ((frames[joint_number].can_id / 16) - 10);
}

//
//  sending message to robot and reading a message
//
// CAN_response_msg_t CanOperations::writeReadMessage()
// {

//     CAN_response_msg_t result_msg;

//     for (int joint_number = 0; joint_number < JOINTS_NUMBER; joint_number++)
//     {
//         // RCLCPP_INFO_STREAM(rclcpp::get_logger("Can operations"), "toruqe byte 0: "
//         //                                                              << std::hex << (int)frames[joint_number].data[0]);
//         // RCLCPP_INFO_STREAM(rclcpp::get_logger("Can operations"), "toruqe byte 1: " <<
//         //                                                              std::hex << (int)frames[joint_number].data[1]);

//         try
//         {
//             if (write(sock, &frames[joint_number], sizeof(struct can_frame)) != sizeof(struct can_frame))
//                 throw std::logic_error("Write torque error.");
//             std::this_thread::sleep_for(std::chrono::milliseconds(2));
//             result_msg[joint_number] = readMessage(joint_number);
//         }
//         catch (const std::logic_error &e)
//         {
//             std::cerr << e.what() << '\n';
//             // writeEmergencyStop(); // TODO przekazać wyjątek dalej?
//         }
//     }

//     return result_msg;
// }
CAN_response_msg_t CanOperations::writeReadFdMessage()
{
    auto t_current = std::chrono::steady_clock::now();
    CAN_response_msg_t result_msg;
    CAN_single_response_msg_t temp_msg;
    if (write(sock, &frames[0], sizeof(struct canfd_frame)) != sizeof(struct canfd_frame))
        throw std::logic_error("Write torque error.");
    for (int joint_number = 0; joint_number < JOINTS_NUMBER; joint_number++)
    {
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("Can operations"), "toruqe byte 0: "
        //                                                              << std::hex << (int)frames[joint_number].data[0]);
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("Can operations"), "toruqe byte 1: " <<
        //                                                              std::hex << (int)frames[joint_number].data[1]);

        //why
        std::this_thread::sleep_for(std::chrono::microseconds(1));
        try
        {
            // result_msg[joint_number] = readMessage(joint_number);
            temp_msg = readJointMessage(joint_number);
            // std::cout<<"got joint "<<temp_msg.joint_id<<"\t"<<temp_msg.position<<std::endl;
            result_msg[temp_msg.joint_id] = temp_msg;
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("Can operations"), "msg id: "<<temp_msg.joint_id);
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("Can operations"), "msg pos: "<<result_msg[temp_msg.joint_id].position);
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("Can operations"), "msg tq: "<<result_msg[temp_msg.joint_id].torque);
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("Can operations"), "msg status: "<<result_msg.at(temp_msg.joint_id).joint_status);
        }
        catch (const std::logic_error &e)
        {
            std::cerr << e.what() << '\n';
            // writeEmergencyStop(); // TODO przekazać wyjątek dalej?
        }
    }
    // std::cout<<"writeReadFdMessage "<<std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-t_current).count()<<std::endl;
    return result_msg;
}
//
//  preparing message for futher sending - creating frames
//
// bool CanOperations::createMessage(CAN_request_msg_t message)
// {

//     for (size_t joint = 0; joint < JOINTS_NUMBER; joint++)
//     {
//         // int16_t torque = static_cast<int16_t>(message[joint].torque);
//         // RCLCPP_INFO_STREAM(rclcpp::get_logger("Can operations"), "float torque: " << message[joint].torque);
//         int16_t torque = static_cast<int16_t>(message[joint].torque / TORQUE_CONSTANT / gears_ratio[joint] * INT16_MAX / MOTOR_MAX_CURRENT);
//         // RCLCPP_INFO_STREAM(rclcpp::get_logger("Can operations"), "int torque: " << torque);

//         // CAN FRAME WRITE
//         frames[joint].can_id = joint + 10;
//         frames[joint].can_dlc = 8;
//         frames[joint].data[0] = torque >> 8;
//         frames[joint].data[1] = torque;
//         frames[joint].data[2] = 0x00;
//         frames[joint].data[3] = 0x00;
//         frames[joint].data[4] = 0x00;
//         frames[joint].data[5] = 0x00;
//         frames[joint].data[6] = 0x00;
//         frames[joint].data[7] = message[joint].turn_motor;
//     }

//     return true;
// }

bool CanOperations::createBroadcastMessage(CAN_request_msg_t message)
{
    auto t_current = std::chrono::steady_clock::now();
    std::array<int16_t, JOINTS_NUMBER> torques;
    std::array<int16_t, JOINTS_NUMBER> turn_motor;
    // intialize frame to zero
    for (size_t i = 0; i < 16; i++)
    {
        frames[0].data[i] = 0x00;
    }
    for (size_t joint = 0; joint < JOINTS_NUMBER; joint++)
    {
        // int16_t torque = static_cast<int16_t>(message[joint].torque);
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("Can operations"), "float torque: " << message[joint].torque);
        try
        {
            torques.at(joint) = static_cast<int16_t>(message[joint].torque);
            // if (torques.at(joint) == 0)
            // {
            //     torques.at(joint)++;
            // }
            // std::cout<<"sending tq: "<<torques.at(joint)<<std::endl;
            turn_motor.at(joint) = message[joint].turn_motor;
            frames[0].data[2 * joint] = torques.at(joint) >> 8;
            frames[0].data[2 * joint + 1] = torques.at(joint);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        // RCLCPP_INFO_STREAM(rclcpp::get_logger("Can operations"), "int torque: " << torque);
    }
    // CAN FRAME WRITE
    frames[0].can_id = 0x0AA;
    frames[0].len = 16;

    // frames[0].data[0] = torques.at(0) >> 8;
    // frames[0].data[1] = torques.at(0);

    // frames[0].data[2] = torques.at(1) >> 8;
    // frames[0].data[3] = torques.at(1);

    // frames[0].data[4] = torques.at(2) >> 8;
    // frames[0].data[5] = torques.at(2);

    // frames[0].data[6] = torques.at(3) >> 8;
    // frames[0].data[7] = torques.at(3);

    // frames[0].data[8] = torques.at(4) >> 8;
    // frames[0].data[9] = torques.at(4);

    // frames[0].data[10] = torques.at(5) >> 8;
    // frames[0].data[11] = torques.at(5);

    frames[0].data[12] = 0x00;

    // TODO0ames[joint].data[7] = message[joint].turn_motor;

    // frames[0].data[13] = 0x11;

    if (JOINTS_NUMBER == 1)
    {
        frames[0].data[13] = turn_motor.at(0) << 4;
    }
    else if (JOINTS_NUMBER == 2)
    {
        frames[0].data[13] = ((turn_motor.at(0) << 4) ^ turn_motor.at(1));
    }
    else if (JOINTS_NUMBER == 3)
    {
        frames[0].data[13] = ((turn_motor.at(0) << 4) ^ turn_motor.at(1));
        frames[0].data[14] = turn_motor.at(2) << 4;
    }
    else if (JOINTS_NUMBER == 4)
    {
        frames[0].data[13] = ((turn_motor.at(0) << 4) ^ turn_motor.at(1));
        frames[0].data[14] = ((turn_motor.at(2) << 4) ^ turn_motor.at(3));
    }
    else if (JOINTS_NUMBER == 5)
    {
        frames[0].data[13] = ((turn_motor.at(0) << 4) ^ turn_motor.at(1));
        frames[0].data[14] = ((turn_motor.at(2) << 4) ^ turn_motor.at(3));
        frames[0].data[15] = turn_motor.at(4) << 4;
    }
    else if (JOINTS_NUMBER >= 6)
    {
        frames[0].data[13] = ((turn_motor.at(0) << 4) ^ turn_motor.at(1));
        frames[0].data[14] = ((turn_motor.at(2) << 4) ^ turn_motor.at(3));
        frames[0].data[15] = ((turn_motor.at(4) << 4) ^ turn_motor.at(5));
    }
    // std::cout<<"createBroadcastMessage "<<std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-t_current).count()<<std::endl;
    return true;
}

//
//  reading function that read message from CAN and create a response structure
//
// CAN_single_response_msg_t CanOperations::readMessage(const int joint)
// {
//     int nbytes = -1;
//     // RCLCPP_INFO_STREAM(rclcpp::get_logger("Can operations"), "can id: " << frames[joint].can_id);
//     nbytes = read(sock, &frames[joint], sizeof(struct can_frame));
//     if (nbytes == -1)
//         throw std::logic_error("Read error. No frame obtained.");

//     // int16_t position_temp = last_postion_msg_value[joint];
//     // int16_t torque_temp = last_torque_msg_value[joint];
//     // int8_t motor_status_temp = last_motor_msg_value[joint];

//     CAN_single_response_msg_t result;

//     if (joint == getFrameId(joint))
//     {
//         int16_t position_temp = static_cast<int16_t>((frames[joint].data[0] << 8) ^ (frames[joint].data[1]));
//         int16_t torque_temp = static_cast<int16_t>((frames[joint].data[2] << 8) ^ (frames[joint].data[3]));
//         int8_t motor_status_temp = static_cast<int8_t>(frames[joint].data[7]);
//         int8_t past_error = static_cast<int8_t>(frames[joint].data[5]);
//         int8_t active_error = static_cast<int8_t>(frames[joint].data[6]);

//         last_postion_msg_value[joint] = position_temp;
//         // last_torque_msg_value[joint] = torque_temp;
//         // last_motor_msg_value[joint] = motor_status_temp;

//         // set obtained form CAN msg values of position and torque
//         result.position = static_cast<int32_t>(position_temp);
//         result.torque = static_cast<int32_t>(torque_temp);
//         result.motor_status = static_cast<int32_t>(motor_status_temp);
//         result.past_error = past_error;
//         result.active_error = active_error;

//         // if (motor_error != 0)
//         // {
//         //     SendAck();
//         // }

//         if (motor_status_temp == 10 || motor_status_temp == 11)
//             RCLCPP_INFO(rclcpp::get_logger("Can operations"), "emergency stop appear after STM ERROR.");
//         // printf("emergency stop appear after STM ERROR.");
//         // throw std::logic_error("emergency stop appear after STM ERROR.");
//     }
//     else
//         throw std::logic_error("Read error. Wrong frame id according to joint number.");

//     if (joint == 0)
//     {

//         pos_can << static_cast<double>(last_postion_msg_value[0] * 2 * M_PI / GEAR_CONST / gears_ratio[0]) << "\n";
//         pos_can.flush();
//     }

//     return result;

//     // result.position = static_cast<double>(position_temp * 2 * M_PI / GEAR_CONST / gears_ratio[joint]);
//     // result.torque = static_cast<double>(torque_temp * TORQUE_CONSTANT * gears_ratio[joint] * MOTOR_MAX_CURRENT / INT16_MAX);
// }

CAN_single_response_msg_t CanOperations::readJointMessage(int joint)
{
    auto t_current = std::chrono::steady_clock::now();
    int nbytes = -1;

    nbytes = read(sock, &frames[joint], sizeof(struct canfd_frame));

    if (nbytes == -1)
        throw std::logic_error("Read error. No frame obtained.");
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("readJointMessage"), " id : "<< frames[joint].can_id);

    // int16_t position_temp = last_postion_msg_value[joint];
    // int16_t torque_temp = last_torque_msg_value[joint];
    // int8_t motor_status_temp = last_motor_msg_value[joint];

    CAN_single_response_msg_t result;
    //useless
    //if (joint == getFrameId(joint))
    if (1)
    {
        int16_t position_temp = static_cast<int16_t>((frames[joint].data[0] << 8) ^ (frames[joint].data[1]));
        int16_t torque_temp = static_cast<int16_t>((frames[joint].data[2] << 8) ^ (frames[joint].data[3]));
        uint8_t temperature_temp = static_cast<uint8_t>((frames[joint].data[4]));
        int16_t error_code = static_cast<int16_t>(frames[joint].data[5]);
        int16_t joint_status = static_cast<int16_t>(frames[joint].data[6]);

        last_postion_msg_value[joint] = position_temp;
        // last_torque_msg_value[joint] = torque_temp;
        // last_motor_msg_value[joint] = motor_status_temp;

        // set obtained form CAN msg values of position and torque
        result.joint_id = ((frames[joint].can_id / 16) - 10); //?
        result.position = position_temp;
        result.torque = torque_temp;
        result.temperature = temperature_temp;
        result.joint_status = joint_status;
        result.error_code = error_code;

        // std::cout<<result.joint_id<<std::endl<<result.position<<std::endl<<result.torque<<std::endl<<result.temperature<<std::endl<<result.joint_status<<std::endl<<result.error_code<<std::endl;
        // result.active_error = active_error;

        // if (motor_error != 0)
        // {
        //     SendAck();
        // }
        if (error_code != 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("Can operations"), "emergency stop appear after STM ERROR.");
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Can operations"), "Error code: " << error_code);
            if (joint_status == JointErrorCode::PAST_ERROR)
                RCLCPP_INFO(rclcpp::get_logger("Can operations"), "Past STM ERROR.");
            else if (joint_status == JointErrorCode::ACTIVE_ERROR)
                RCLCPP_INFO(rclcpp::get_logger("Can operations"), "Active STM ERROR.");
        }
        // printf("emergency stop appear after STM ERROR.");
        // throw std::logic_error("emergency stop appear after STM ERROR.");
    }
    else
        throw std::logic_error("Read error. Wrong frame id according to joint number.");

    // if (joint == 0)
    // {

    //     pos_can << static_cast<double>(last_postion_msg_value[0] * 2 * M_PI / GEAR_CONST / gears_ratio[0]) << "\n";
    //     pos_can.flush();
    // }
    // std::cout<<"readJointMessage "<<std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-t_current).count()<<std::endl;
    return result;

    // result.position = static_cast<double>(position_temp * 2 * M_PI / GEAR_CONST / gears_ratio[joint]);
    // result.torque = static_cast<double>(torque_temp * TORQUE_CONSTANT * gears_ratio[joint] * MOTOR_MAX_CURRENT / INT16_MAX);
}

//
//  sending zero values and read positions
//  use at the beginnig or when robot motors are turn off
//
// CAN_response_msg_t CanOperations::readOnlyPosition()
// {
//     CAN_response_msg_t result_msg;
//     CAN_request_msg_t null_msg;

//     // prepare null message
//     for (int joint = 0; joint < JOINTS_NUMBER; joint++)
//     {
//         null_msg[joint].torque = 0;
//         null_msg[joint].turn_motor = 2;
//     }

//     // create null message
//     createMessage(null_msg);

//     // send message and read position
//     for (int joint = 0; joint < JOINTS_NUMBER; joint++)
//     {
//         if (write(sock, &frames[joint], sizeof(struct can_frame)) != sizeof(struct can_frame))
//             throw std::logic_error("Write torque error.");
//         // perror("Write/Read posi tion error.");

//         std::this_thread::sleep_for(std::chrono::milliseconds(2));

//         result_msg[joint] = readMessage(joint);
//     }

//     return result_msg;
// }
// void CanOperations::SendAck()
// {
//     // CAN_response_msg_t result_msg;
//     CAN_request_msg_t null_msg;

//     // prepare null message
//     for (int joint = 0; joint < JOINTS_NUMBER; joint++)
//     {
//         null_msg[joint].torque = 0;
//         null_msg[joint].turn_motor = 2;
//     }

//     // create null message
//     createMessage(null_msg);

//     // send message and read position
//     for (int joint = 0; joint < JOINTS_NUMBER; joint++)
//     {
//         if (write(sock, &frames[joint], sizeof(struct can_frame)) != sizeof(struct can_frame))
//             throw std::logic_error("Write torque error.");
//         // perror("Write/Read posi tion error.");

//         std::this_thread::sleep_for(std::chrono::milliseconds(2));

//         // result_msg[joint] = readMessage(joint);
//     }

//     // return result_msg;
// }

//
// old functions
//

// TODO rewrite with throw exceptions
bool CanOperations::initSocket()
{
    if ((sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("Socket");
        return false;
    }
    else
    {
        printf("CAN SOCKET CREATED \n");
    }
    strcpy(ifr.ifr_name, CAN_NUMBER);
    ioctl(sock, SIOCGIFINDEX, &ifr);
    fcntl(sock, F_SETFL, O_NONBLOCK);
    memset(&addr, 0, sizeof(addr));

    error_appear = false;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    int enable_canfd = 1;
    if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                   &enable_canfd, sizeof(enable_canfd)))
    {
        printf("error when enabling CAN FD support\n");
        return 1;
    }

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind");
        return false;
    }
    else
    {
        printf("CAN BINDED \n");
        return true;
    }
}

bool CanOperations::restart(int joint_number)
{
    // CAN FRAME WRITE
    frame.can_id = joint_number + 10;
    frame.can_dlc = 8;
    frame.data[0] = 0xFF;
    frame.data[1] = 0xFF;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = 0xFF;
    frame.data[7] = 0x02;

    if (write(sock, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write/Read position error.");
        return false;
    }
    return true;
}

// TODO rewrite to frames
// CAN_response_msg_t CanOperations::writeEmergencyStop()
// {
//     CAN_response_msg_t result_msg;
//     CAN_request_msg_t null_msg;

//     // prepare null message
//     for (int joint = 0; joint < JOINTS_NUMBER; joint++)
//     {
//         null_msg[joint].torque = 0;
//         null_msg[joint].turn_motor = 0;
//     }

//     // create null message
//     createMessage(null_msg);

//     // send message and read position
//     for (int joint = 0; joint < JOINTS_NUMBER; joint++)
//     {
//         if (write(sock, &frames[joint], sizeof(struct can_frame)) != sizeof(struct can_frame))
//             throw std::logic_error("Write torque error.");
//         // perror("Write/Read posi tion error.");

//         std::this_thread::sleep_for(std::chrono::milliseconds(INTERVAL));

//         result_msg[joint] = readMessage(joint);
//     }

//     return result_msg;
// }
