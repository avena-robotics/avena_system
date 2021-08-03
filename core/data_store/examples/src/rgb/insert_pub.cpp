#include "custom_interfaces/srv/data_store_rgb_data_select.hpp"

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    using RgbDataSelect = custom_interfaces::srv::DataStoreRgbDataSelect;

    MinimalPublisher()
        : Node("minimal_publisher"), count_(0)
    {
        rclcpp::QoS qos_latching = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        publisher_ = this->create_publisher<RgbDataSelect::Response>("rgb_data",qos_latching);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = RgbDataSelect::Response();
        message.time_stamp.data = count_++;
        message.camera_1_rgb = sensor_msgs::msg::Image();
        message.camera_2_rgb = sensor_msgs::msg::Image();
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<RgbDataSelect::Response>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}