#ifndef SECURITY_RGB_HPP
#define SECURITY_RGB_HPP
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <tuple>
#include <thread>
#include <iostream>
#include "std_msgs/msg/bool.hpp"
#include "helpers_commons/helpers_commons.hpp"
#include <atomic>
#include <csignal>
namespace security
{
    using Bool = std_msgs::msg::Bool;
    class SecurityRgb : public rclcpp::Node, public helpers::WatchdogInterface
    {
    public:
        explicit SecurityRgb(const rclcpp::NodeOptions &options);
        ~SecurityRgb();
        virtual void initNode() override;
        virtual void shutDownNode() override;

    private:
        helpers::Watchdog::SharedPtr _watchdog;

        rclcpp::Subscription<Bool>::SharedPtr _danger_tool_in_hand_sub;
        rclcpp::Subscription<Bool>::SharedPtr _security_trigger_sub;
        rclcpp::Publisher<Bool>::SharedPtr _gui_warning_pub;
        rclcpp::Publisher<Bool>::SharedPtr _security_pause_pub;

        std::atomic<bool> _danger_tool_in_hand{false};
        std::atomic<bool> _security_trigger{false};

        std::thread _run_thread;
        std::mutex _status_mutex;

        std::tuple<bool, bool, size_t, size_t> _truth_table(bool dt, bool st, size_t count1, size_t count2);
        void _run();
        static void _signalHandler(int signum);
    };

} // namespace security

#endif