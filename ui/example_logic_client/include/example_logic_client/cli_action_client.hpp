#ifndef CLI__CLI_ACTION_CLIENT_HPP_
#define CLI__CLI_ACTION_CLIENT_HPP_

// ___CPP___
#include <chrono>

// ___ROS2___
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "rclcpp_components/register_node_macro.hpp"

// ___Avena___
#include "custom_interfaces/action/simple_action.hpp"
#include "cli/visibility_control.h"
#include "cli/util.hpp"

namespace cli
{
  using namespace std::chrono_literals;

  class CliActionClient : public rclcpp::Node
  {
  public:
    using SimpleAction = custom_interfaces::action::SimpleAction;
    using GoalHandleSimpleAction = rclcpp_action::ClientGoalHandle<SimpleAction>;

    CLI_PUBLIC
    explicit CliActionClient(std::string action_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    CLI_PUBLIC
    virtual ~CliActionClient(){};

    CLI_PUBLIC
    int sendGoal();

  private:
    // ___Member functions___
    CLI_LOCAL
    int _waitForServer();

    CLI_LOCAL
    int _getActionResult(std::shared_future<std::shared_ptr<GoalHandleSimpleAction>> goal_future);

    // ___Attributes___
    rclcpp_action::Client<SimpleAction>::SharedPtr _action_client;
    std::string _action_name;
    Timer::SharedPtr _execution_timer;
  };

} // namespace cli

#endif // CLI__CLI_LIB_HPP_
