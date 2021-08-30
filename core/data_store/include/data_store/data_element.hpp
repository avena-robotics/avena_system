#ifndef _DATA_ELEMENT_HPP
#define _DATA_ELEMENT_HPP
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
// #include <optional>
namespace data_store
{
    template <typename SELECT, typename INSERT, typename GETLIST, typename DELETE>
    class DataElement
    {
    private:
        using String = std_msgs::msg::String;
        // std::unordered_map<double, typename SELECT::Response> _data_element_container;
        std::map<double, typename SELECT::Response> _data_element_container;
        std::mutex _container_mutex;

        void _select(const std::shared_ptr<typename SELECT::Request> request,
                     std::shared_ptr<typename SELECT::Response> response);

        void _insert(const std::shared_ptr<typename INSERT::Request> request,
                     std::shared_ptr<typename INSERT::Response> response);

        void _getList(const std::shared_ptr<typename GETLIST::Request> request,
                      std::shared_ptr<typename GETLIST::Response> response);

        void _delete(const std::shared_ptr<typename DELETE::Request> request,
                     std::shared_ptr<typename DELETE::Response> response);

        typename rclcpp::Service<SELECT>::SharedPtr _select_server;
        typename rclcpp::Service<INSERT>::SharedPtr _insert_server;
        typename rclcpp::Service<GETLIST>::SharedPtr _get_list_server;
        typename rclcpp::Service<DELETE>::SharedPtr _delete_server;

        rclcpp::Publisher<String>::SharedPtr _change_flag_publisher;
        String _change_flag_message;

        // typename rclcpp::Subscription<typename SELECT::Response>::SharedPtr _insert_sub;
        // std::optional<typename SELECT::Response> _msg;
        // void _insert_cb(const typename SELECT::Response::SharedPtr input_msg_ptr);

    public:
        // DataElement(rclcpp::Node::SharedPtr node_ptr, const rclcpp::QoS qos, const std::string data_element_name);
        DataElement(rclcpp::Node::SharedPtr node_ptr, const std::string data_element_name);
        ~DataElement() = default;
    };
} // end of data_store
#include "data_element_impl.hpp"
#endif