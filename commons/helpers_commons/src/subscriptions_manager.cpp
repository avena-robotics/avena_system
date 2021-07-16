#include "helpers_commons/subscriptions_manager.hpp"

namespace helpers
{
    SubscriptionsManager::SubscriptionsManager(const rclcpp::Node::SharedPtr node)
        : SubscriptionsManager(node->get_node_topics_interface())
    {
    }

    SubscriptionsManager::SubscriptionsManager(const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface)
        : _node_topics_interface(node_topics_interface)
    {
        _node_name = _node_topics_interface->get_node_base_interface()->get_name();
    }

    SubscriptionsManager::~SubscriptionsManager() {}

    int SubscriptionsManager::createSubscription(const std::string &topic_name, const std::string &topic_type, const rclcpp::QoS &qos)
    {
        auto subscription = std::shared_ptr<GenericSubscription>();
        try
        {
            auto library_generic_subscriptor = rosbag2_cpp::get_typesupport_library(topic_type, "rosidl_typesupport_cpp");
            auto type_support = rosbag2_cpp::get_typesupport_handle(topic_type, "rosidl_typesupport_cpp", library_generic_subscriptor);
            subscription = std::make_shared<GenericSubscription>(
                _node_topics_interface->get_node_base_interface(),
                // _node->get_node_base_interface().get(),
                *type_support,
                topic_name,
                qos,
                [this, topic_name](const std::shared_ptr<rclcpp::SerializedMessage> message) {
                    // RCLCPP_INFO_STREAM(rclcpp::get_logger(_node_name), "Received message from \"" << topic_name << "\" topic");
                    _messages[topic_name] = message;
                });
            // _node->get_node_topics_interface()->add_subscription(subscription, nullptr);
            _node_topics_interface->add_subscription(subscription, nullptr);
        }
        catch (const std::runtime_error &ex)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(_node_name), "Error occured while creating subscription for \"" << topic_name << "\" topic with type \"" << topic_type << "\"");
            return 1;
        }
        _subscriptions[topic_name] = subscription;
        RCLCPP_INFO_STREAM(rclcpp::get_logger(_node_name), "Created subscription for \"" << topic_name << "\" topic with type \"" << topic_type << "\"");
        return 0;
    }

} // namespace helpers
