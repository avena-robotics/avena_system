#include "logic_bt/security_rgb.hpp"

std::tuple<bool, bool, size_t, size_t> truth_table(bool dt, bool st, size_t count1, size_t count2)
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
class Subscribers
{
public:
    static bool danger_tool_in_hand, security_trigger;
    static void danger_tool_in_hand_callback(const Bool::SharedPtr msg)
    {
        if (msg)
        {
            danger_tool_in_hand = msg->data;
        }
    }
    static void security_trigger_callback(const Bool::SharedPtr msg)
    {
        if (msg)
        {
            security_trigger = msg->data;
        }
    }
};
bool Subscribers::danger_tool_in_hand = false;
bool Subscribers::security_trigger = false;
void signalHandler(int signum)
{
    rclcpp::shutdown();
    std::cout << signum << std::endl;
    // supposed to be signum
    exit(0);
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, signalHandler);
    auto node_ptr = rclcpp::Node::make_shared("security_rgb");
    auto gui_warning_pub = node_ptr->create_publisher<Bool>("gui_warning", 1);
    auto security_pause_pub = node_ptr->create_publisher<Bool>("security_pause", 1);
    auto danger_tool_in_hand_sub = node_ptr->create_subscription<Bool>(
        "danger_tool_in_hand", 1, std::bind(&Subscribers::danger_tool_in_hand_callback, std::placeholders::_1));
    auto security_trigger_sub = node_ptr->create_subscription<Bool>(
        "security_trigger", 1, std::bind(&Subscribers::security_trigger_callback, std::placeholders::_1));
    auto security_pause_msg = Bool();
    security_pause_msg.data = false;
    auto gui_warning_msg = Bool();
    gui_warning_msg.data = false;
    size_t count1 = 0;
    size_t count2 = 0;
    bool sp{false}, gw{false};
    while (true)
    {
        std::tie(sp, gw, count1, count2) = truth_table(Subscribers::danger_tool_in_hand, Subscribers::security_trigger, count1, count2);
        security_pause_msg.data = sp;
        security_pause_pub->publish(security_pause_msg);
        gui_warning_msg.data = gw;
        gui_warning_pub->publish(gui_warning_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
        rclcpp::spin_some(node_ptr);
    }
    rclcpp::shutdown();
    return 0;
}
