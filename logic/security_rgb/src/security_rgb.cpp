#include "security_rgb/security_rgb.hpp"

namespace security
{
    SecurityRgb::SecurityRgb(const rclcpp::NodeOptions &options) : Node("security_rgb", options)
    {
        status = custom_interfaces::msg::Heartbeat::STOPPED;
        RCLCPP_INFO(this->get_logger(), "started security_rgb Node");
        _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
    }
    SecurityRgb::~SecurityRgb()
    {
        if (_run_thread.joinable())
        {
            _run_thread.join();
        }
    }
    void SecurityRgb::initNode()
    {
        status = custom_interfaces::msg::Heartbeat::STARTING;
        RCLCPP_INFO(get_logger(), "Initialization of security_rgb.");
        _security_trigger_sub = create_subscription<Bool>("security_trigger", 1, [this](const typename Bool::SharedPtr msg) -> void
                                                          { _security_trigger = msg->data; });
        _danger_tool_in_hand_sub = create_subscription<Bool>("danger_tool_in_hand", 1, [this](const typename Bool::SharedPtr msg) -> void
                                                             { _danger_tool_in_hand = msg->data; });
        _gui_warning_pub = create_publisher<Bool>("gui_warning", 1);
        _security_pause_pub = create_publisher<Bool>("security_pause", 1);
        signal(SIGINT, _signalHandler);
        _run_thread = std::thread(std::bind(&SecurityRgb::_run, this));
        status = custom_interfaces::msg::Heartbeat::RUNNING;
    }
    void SecurityRgb::shutDownNode()
    {
        RCLCPP_INFO(this->get_logger(), "shut Down security_rgb Node");
        if (status != custom_interfaces::msg::Heartbeat::STOPPED)
            status = custom_interfaces::msg::Heartbeat::STOPPED;
        if (_run_thread.joinable())
        {
            _run_thread.join();
        }
    }
    std::tuple<bool, bool, size_t, size_t> SecurityRgb::_truth_table(bool dt, bool st, size_t count1, size_t count2)
    {
        size_t max_count1 = 151; // 5 sec
        size_t max_count2 = 333; // 10 sec
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("case"), "count 1: " << count1);
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("case"), "count 2: " << count2);

        if (!st) //
        {
            // RCLCPP_INFO(rclcpp::get_logger("case"), "case 1");
            return std::tuple<bool, bool, size_t, size_t>(false, false, 0, 0);
        }
        else if (!dt && st && count1 < max_count1 && count2 < max_count2)
        {
            // RCLCPP_INFO(rclcpp::get_logger("case"), "case 2");

            return std::tuple<bool, bool, size_t, size_t>(true, true, ++count1, ++count2);
        }
        else if (!dt && st && count1 >= max_count1 && count2 < max_count2)
        {
            // RCLCPP_INFO(rclcpp::get_logger("case"), "case 3");

            return std::tuple<bool, bool, size_t, size_t>(false, true, ++count1, ++count2);
        }
        else if (!dt && st && count1 >= max_count1 && count2 >= max_count2)
        {
            // RCLCPP_INFO(rclcpp::get_logger("case"), "case 4");

            return std::tuple<bool, bool, size_t, size_t>(false, false, ++count1, ++count2);
        }
        else if (dt && st && count1 < max_count1 && count2 < max_count2)
        {
            // RCLCPP_INFO(rclcpp::get_logger("case"), "case 5");

            return std::tuple<bool, bool, size_t, size_t>(true, true, ++count1, ++count2);
        }
        else if (dt && st && count1 >= max_count1 && count2 < max_count2)
        {
            // RCLCPP_INFO(rclcpp::get_logger("case"), "case 6");

            return std::tuple<bool, bool, size_t, size_t>(true, true, ++count1, ++count2);
        }
        else if (dt && st && count1 >= max_count1 && count2 >= max_count2)
        {
            // RCLCPP_INFO(rclcpp::get_logger("case"), "case 7");

            return std::tuple<bool, bool, size_t, size_t>(true, true, ++count1, ++count2);
        }
        else if (!dt && st && count1 < max_count1 && count2 >= max_count2)
        {
            // RCLCPP_INFO(rclcpp::get_logger("case"), "case 8");

            return std::tuple<bool, bool, size_t, size_t>(false, false, ++count1, ++count2);
        }
        else if (dt && st && count1 < max_count1 && count2 >= max_count2)
        {
            // RCLCPP_INFO(rclcpp::get_logger("case"), "case 9");

            return std::tuple<bool, bool, size_t, size_t>(true, true, ++count1, ++count2);
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("case"), "false case");

            throw std::runtime_error(std::string("Undeifned case in else"));
            return std::tuple<bool, bool, size_t, size_t>(false, false, 0, 0);
        }
    }
    void SecurityRgb::_run()
    {
        size_t count1 = 0;
        size_t count2 = 0;
        bool sp{false}, gw{false};
        Bool security_pause_msg, gui_warning_msg;
        security_pause_msg.data = false;
        gui_warning_msg.data = false;
        std::lock_guard<std::mutex> lock(_status_mutex);
        while (status != custom_interfaces::msg::Heartbeat::STOPPED)
        {
            std::tie(sp, gw, count1, count2) = _truth_table(_danger_tool_in_hand, _security_trigger, count1, count2);
            security_pause_msg.data = sp;
            _security_pause_pub->publish(security_pause_msg);
            gui_warning_msg.data = gw;
            _gui_warning_pub->publish(gui_warning_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
    }
    void SecurityRgb::_signalHandler(int signum)
    {
        rclcpp::shutdown();
        exit(signum);
    }

} // namespace security
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(security::SecurityRgb)
