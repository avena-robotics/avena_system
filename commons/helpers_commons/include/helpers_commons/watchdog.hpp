#ifndef HELPERS_COMMONS__WATCHDOG_HPP_
#define HELPERS_COMMONS__WATCHDOG_HPP_

// ___Avena___
#include "custom_interfaces/msg/watchdog.hpp"
#include "custom_interfaces/msg/heartbeat.hpp"
#include "custom_interfaces/msg/statistics.hpp"
#include "custom_interfaces/msg/module_command.hpp"
#include "helpers_commons/proc_stat.hpp"

// ___CPP___
#include <optional>
#include <chrono>

// ___ROS2___
#include <rclcpp/rclcpp.hpp>

namespace helpers
{
  using namespace std::chrono_literals;

  //every watchable node class should implement this interface
  class WatchdogInterface
  {
  public:
    virtual void initNode() = 0;     //push start button
    virtual void shutDownNode() = 0; //push stop button
    int status;
  };

  class BTInterface
  {
  public:
    virtual void run() = 0;
  };

  class Watchdog
  {
  public:
    using UniquePtr = std::unique_ptr<Watchdog>;
    using SharedPtr = std::shared_ptr<Watchdog>;

    explicit Watchdog(rclcpp::Node *node, helpers::WatchdogInterface *gui_interface, const std::string &system_monitor_name);
    Watchdog(const Watchdog &other) = delete;
    ~Watchdog();
    // Watchdog &operator=(const Watchdog &other) = delete;
    bool isWorking();
    bool isActionStarted();
    void beginAction();
    void endAction();

  private:
    void _watchdogCallback(const custom_interfaces::msg::Watchdog::SharedPtr watchdog_data);
    void _systemCheckTimerCallback();
    void _broadcastHeartbeat();
    void _broadcastStatistics();
    void _commandCallback(const custom_interfaces::msg::ModuleCommand::SharedPtr command);

    void _spinHeartBeat();
    void _spinStatistics();

    rclcpp::Node *_node;
    helpers::WatchdogInterface *_gui_interface;
    rclcpp::Subscription<custom_interfaces::msg::Watchdog>::SharedPtr _sub_watchdog;
    rclcpp::Subscription<custom_interfaces::msg::ModuleCommand>::SharedPtr _sub_command;

    rclcpp::Publisher<custom_interfaces::msg::Heartbeat>::SharedPtr _pub_heartbeat;
    rclcpp::Publisher<custom_interfaces::msg::Statistics>::SharedPtr _pub_statistics;

    rclcpp::TimerBase::SharedPtr _pub_heartbeat_timer;
    rclcpp::TimerBase::SharedPtr _pub_statistics_timer;
    rclcpp::TimerBase::SharedPtr _system_check_timer;

    std::chrono::duration<double> _pub_heartbeat_duration;
    std::chrono::duration<double> _pub_statistics_duration;

    std::chrono::duration<double> _system_check_duration;

    std::string _system_monitor_name;
    std::string _node_name;
    bool _working;

    bool _action_started;
    std::chrono::time_point<std::chrono::system_clock> _start_action_timestamp;
    uint32_t _action_time_period;
    double _prev_cpu_jiffies{0.0};
    double _prev_proc_jiffies{0.0};

    std::thread _heart_beat_thread;
    std::thread _statistics_thread;
  };

} // namespace helpers

#endif // HELPERS_COMMONS__WATCHDOG_HPP_
