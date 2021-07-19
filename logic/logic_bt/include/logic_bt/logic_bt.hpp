#ifndef _LOGIC_BT_HPP
#define LOGIC_BT_HPP

#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "helpers_commons/helpers_commons.hpp"
#include "custom_interfaces/action/bt_pick_and_place_action.hpp"
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "timer.hpp"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include <filesystem>
#include <csignal>



// #include "custom_interfaces/msg/heartbeat.hpp"
// #include "helpers_commons/helpers_commons.hpp"
using Milliseconds = std::chrono::milliseconds;
namespace logic_bt
{
    // class Logic : public rclcpp::Node, public helpers::WatchdogInterface
    class Logic : public rclcpp::Node, public helpers::WatchdogInterface
    {
    private:
        using PickAndPlaceAction = custom_interfaces::action::BTPickAndPlaceAction;
        using GoalHandlePickAndPlaceAction = rclcpp_action::ServerGoalHandle<PickAndPlaceAction>;
        BT::Tree _logic_tree;
        helpers::Watchdog::SharedPtr _watchdog;
        std::shared_ptr<BT::PublisherZMQ> _publisher_zmq_ptr;
        rclcpp_action::Server<PickAndPlaceAction>::SharedPtr logic_server_;
        rclcpp_action::GoalResponse _handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const PickAndPlaceAction::Goal> goal);
        rclcpp_action::CancelResponse _handle_cancel(
            const std::shared_ptr<GoalHandlePickAndPlaceAction> goal_handle);

        void _handle_accepted(const std::shared_ptr<GoalHandlePickAndPlaceAction> goal_handle);

        void _execute(const std::shared_ptr<GoalHandlePickAndPlaceAction> goal_handle);
        const std::array<std::string, 4> _commands{"start", "pause", "resume", "stop"};

        const std::unordered_map<BT::NodeStatus, std::string> _bt_states = {{BT::NodeStatus::RUNNING, "running"},
                                                                            {BT::NodeStatus::IDLE, "idle"},
                                                                            {BT::NodeStatus::SUCCESS, "success"},
                                                                            {BT::NodeStatus::FAILURE, "failure"}};
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _timer_pub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _timer_sub;
        void _timer_callback(const std_msgs::msg::String::SharedPtr msg);
        Timer _timer;
        BT::BehaviorTreeFactory _factory;
        static void _signalHandler(int signum);


    public:
        explicit Logic(const rclcpp::NodeOptions &options);
        ~Logic();
        virtual void initNode() override;
        virtual void shutDownNode() override;
        // void initNode();
        // void shutDownNode();
        // virtual void run() override;
    };
} // end of logic_bt
#endif
