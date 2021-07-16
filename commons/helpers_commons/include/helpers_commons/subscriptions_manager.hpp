#ifndef HELPERS_COMMONS__SUBSCRIPTIONS_MANAGER_HPP_
#define HELPERS_COMMONS__SUBSCRIPTIONS_MANAGER_HPP_

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>

// ___Package___
#include "helpers_commons/generic_subscription.hpp"

namespace helpers
{
  class SubscriptionsManager
  {
  public:
    explicit SubscriptionsManager(const rclcpp::Node::SharedPtr node);
    explicit SubscriptionsManager(const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface);
    // SubscriptionsManager(const SubscriptionsManager &other) = delete;
    virtual ~SubscriptionsManager();

    int createSubscription(const std::string &topic_name, const std::string &topic_type, const rclcpp::QoS &qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    template <typename MessageT>
    std::shared_ptr<MessageT> getData(const std::string &topic_name)
    {
      std::shared_ptr<MessageT> output_data;
      if (_messages[topic_name])
      {
        try
        {
          auto serializer = rclcpp::Serialization<MessageT>();
          output_data = std::make_shared<MessageT>();
          serializer.deserialize_message(_messages[topic_name].get(), output_data.get());
        }
        catch (const std::exception &e)
        {
          RCLCPP_ERROR(rclcpp::get_logger(_node_name), "Error deserializing message");
          output_data.reset();
        }
      }
      return output_data;
    }

    // TODO: Needs to be checked
    // template <typename MessageT>
    // int isHeaderValid(const std::shared_ptr<MessageT> &msg)
    // {
    //     if (msg->header.stamp == builtin_interfaces::msg::Time())
    //         return 1;
    //     return 0;
    // }

    using SharedPtr = std::shared_ptr<SubscriptionsManager>;
    using UniquePtr = std::unique_ptr<SubscriptionsManager>;

  private:
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr _node_topics_interface;
    std::string _node_name;
    std::map<std::string, std::shared_ptr<GenericSubscription>> _subscriptions;
    std::map<std::string, std::shared_ptr<rclcpp::SerializedMessage>> _messages;
  };

} // namespace helpers

#endif // HELPERS_COMMONS__SUBSCRIPTIONS_MANAGER_HPP_
