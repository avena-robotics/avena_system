#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "candriver/constants.h"

// #include "custom_interfaces/srv/get_value.hpp"
// #include "custom_interfaces/srv/set_value.hpp"
#include "custom_interfaces/srv/set_arm_torques.hpp"
#include "custom_interfaces/srv/get_arm_state.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_talker");
  rclcpp::Client<custom_interfaces::srv::SetArmTorques>::SharedPtr client =
      node->create_client<custom_interfaces::srv::SetArmTorques>("/arm_controller/set_torques");

  rclcpp::Client<custom_interfaces::srv::GetArmState>::SharedPtr get_client =
      node->create_client<custom_interfaces::srv::GetArmState>("/arm_controller/get_current_arm_state");

  auto request = std::make_shared<custom_interfaces::srv::SetArmTorques::Request>();
  request->torques.resize(JOINTS_NUMBER);
  request->turn_motor.resize(JOINTS_NUMBER);
  for (size_t jnt_idx = 0; jnt_idx < JOINTS_NUMBER; jnt_idx++)
  {
    // request->device_name = 0;
    request->torques.at(jnt_idx) = 12;
    request->turn_motor.at(jnt_idx) = 1;
  }
  auto get_request = std::make_shared<custom_interfaces::srv::GetArmState::Request>();

  while (!client->wait_for_service(1s) && !get_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {

      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  while (true)
  {
    auto result = client->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    { 
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_torques"); // CHANGE
    }
    auto get_result = get_client->async_send_request(get_request);
    if (rclcpp::spin_until_future_complete(node, get_result) == rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      for (size_t jnt_idx = 0; jnt_idx < JOINTS_NUMBER; jnt_idx++)
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "position: %f", get_result.get()->arm_current_positions.at(jnt_idx));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "torque: %f", get_result.get()->arm_current_torques.at(jnt_idx));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "status: %f", get_result.get()->arm_current_status.at(jnt_idx));
      }
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_arm_state"); // CHANGE
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  rclcpp::shutdown();
  return 0;
}
