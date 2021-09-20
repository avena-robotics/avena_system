#ifndef _DATA_ELEMENT_IMPL_HPP
#define _DATA_ELEMENT_IMPL_HPP
namespace data_store
{
    template <typename SELECT, typename INSERT, typename GETLIST, typename DELETE>
    // DataElement<SELECT, INSERT, GETLIST, DELETE>::DataElement(rclcpp::Node::SharedPtr node_ptr, const rclcpp::QoS qos, const std::string data_element_name)
    DataElement<SELECT, INSERT, GETLIST, DELETE>::DataElement(rclcpp::Node::SharedPtr node_ptr, const std::string data_element_name)
    {
        // _insert_sub = node_ptr->create_subscription<typename SELECT::Response>(data_element_name, qos, std::bind(&DataElement::_insert_cb, this, std::placeholders::_1));
        _select_server = node_ptr->create_service<SELECT>(data_element_name + "_select", std::bind(&DataElement::_select, this, std::placeholders::_1, std::placeholders::_2));
        _insert_server = node_ptr->create_service<INSERT>(data_element_name + "_insert", std::bind(&DataElement::_insert, this, std::placeholders::_1, std::placeholders::_2));
        _get_list_server = node_ptr->create_service<GETLIST>(data_element_name + "_get_list", std::bind(&DataElement::_getList, this, std::placeholders::_1, std::placeholders::_2));
        _delete_server = node_ptr->create_service<DELETE>(data_element_name + "_delete", std::bind(&DataElement::_delete, this, std::placeholders::_1, std::placeholders::_2));

        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        _change_flag_publisher = node_ptr->create_publisher<String>(data_element_name + "_change_flag", qos_settings);
        _change_flag_message = String();
        _change_flag_message.data = data_element_name;
    }
    template <typename SELECT, typename INSERT, typename GETLIST, typename DELETE>
    void DataElement<SELECT, INSERT, GETLIST, DELETE>::_insert(const std::shared_ptr<typename INSERT::Request> request,
                                                                               std::shared_ptr<typename INSERT::Response> response)
    {
        std::lock_guard<std::mutex> lock(_container_mutex);
        if (request)
        {
            auto result_msg = std_msgs::msg::Bool();
            try
            {
                // if (_msg)
                if (request)
                {
                    auto container_element = typename SELECT::Response();
                    container_element.time_stamp = request->time_stamp;
                    container_element.data = request->data;
                    // ******************temporary*************************** //
                    _data_element_container.clear();
                    // _data_element_container.insert_or_assign(0.0, _msg.value());
                    _data_element_container.insert_or_assign(1.0, container_element);
                    // ******************temporary*************************** //
                    // _data_element_container.insert_or_assign(request->time_stamp.data, container_element);
                    // _data_element_container.insert_or_assign(_msg.value().time_stamp.data, _msg.value());
                    result_msg.data = true;
                    response->result = result_msg;
                    _change_flag_publisher->publish(_change_flag_message);
                    // _msg.reset();
                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger("debug"), "No data on topic");
                    result_msg.data = false;
                    response->result = result_msg;
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("debug"), e.what());
                result_msg.data = false;
                response->result = result_msg;
            }
        }
    }
    template <typename SELECT, typename INSERT, typename GETLIST, typename DELETE>
    void DataElement<SELECT, INSERT, GETLIST, DELETE>::_delete(const std::shared_ptr<typename DELETE::Request> request,
                                                                               std::shared_ptr<typename DELETE::Response> response)
    {
        std::lock_guard<std::mutex> lock(_container_mutex);
        if (request)
        {
            auto result_msg = std_msgs::msg::Bool();

            if (_data_element_container.erase(request->time_stamp.data) != 0)
            {
                result_msg.data = true;
                response->result = result_msg;
            }

            else
            {
                RCLCPP_WARN(rclcpp::get_logger("debug"), "item is not found, you can not delete it");
                result_msg.data = false;
                response->result = result_msg;
            }
        }
    }
    template <typename SELECT, typename INSERT, typename GETLIST, typename DELETE>
    void DataElement<SELECT, INSERT, GETLIST, DELETE>::_getList(const std::shared_ptr<typename GETLIST::Request> request,
                                                                                std::shared_ptr<typename GETLIST::Response> response)
    {
        std::lock_guard<std::mutex> lock(_container_mutex);
        if (request)
        {
            try
            {
                std_msgs::msg::Float64 timestamp_msg;
                for (const auto &[time_stamp, _] : _data_element_container)
                {
                    timestamp_msg.data = time_stamp;
                    response->time_stamps.emplace_back(timestamp_msg);
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("debug"), e.what());
                response = std::make_shared<typename GETLIST::Response>();
            }
        }
    }
    template <typename SELECT, typename INSERT, typename GETLIST, typename DELETE>
    void DataElement<SELECT, INSERT, GETLIST, DELETE>::_select(const std::shared_ptr<typename SELECT::Request> request,
                                                                               std::shared_ptr<typename SELECT::Response> response)
    {
        std::lock_guard<std::mutex> lock(_container_mutex);
        if (request)
        {
            try
            {
                if (response)
                {
                    // ******************temporary*************************** //
                    *response = _data_element_container.at(1.0);
                    // ******************temporary*************************** //
                    // *response = _data_element_container.at(request->time_stamp.data);
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(rclcpp::get_logger("debug"), "There is no data for requested timestamp");
                response = std::make_shared<typename SELECT::Response>();
            }
        }
    }
    // template <typename SELECT, typename INSERT, typename GETLIST, typename DELETE>
    // void DataElement<SELECT, INSERT, GETLIST, DELETE>::_insert_cb(const typename SELECT::Response::SharedPtr input_msg_ptr)
    // {
    //     if (input_msg_ptr)
    //     {
    //         _msg = *input_msg_ptr;
    //     }
    // }

} // end of data_store

#endif