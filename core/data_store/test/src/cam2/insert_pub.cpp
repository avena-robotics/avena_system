#include "custom_interfaces/srv/data_store_item_cam2_select.hpp"

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
    using ItemCam2Select = custom_interfaces::srv::DataStoreItemCam2Select;

    MinimalPublisher()
        : Node("minimal_publisher"), count_(0)
    {
        rclcpp::QoS qos_latching = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        publisher_ = this->create_publisher<ItemCam2Select::Response>("item_cam2",qos_latching);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = ItemCam2Select::Response();
        // message.camera_1_rgb = sensor_msgs::msg::Image();
        // message.camera_2_rgb = sensor_msgs::msg::Image();
        message.time_stamp.data = count_++;
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ItemCam2Select::Response>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}