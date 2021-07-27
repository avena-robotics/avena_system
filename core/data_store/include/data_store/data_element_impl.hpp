#ifndef _DATA_ELEMENT_IMPL_HPP
#define _DATA_ELEMENT_IMPL_HPP
namespace data_store
{
    template <typename SELECT, typename INSERT, typename GETLIST, typename DELETE>
    DataElement<SELECT, INSERT, GETLIST, DELETE>::DataElement(rclcpp::Node::SharedPtr node_ptr, const rclcpp::QoS qos, const std::string data_element_name)
    {
        using namespace std::placeholders;
        _insert_sub = node_ptr->create_subscription<typename SELECT::Response>(data_element_name, qos, std::bind(&DataElement::_insert_cb, this, _1));
        _select_server = node_ptr->create_service<SELECT>(data_element_name + "_select", std::bind(&DataElement::_select, this, _1, _2));
        _insert_server = node_ptr->create_service<INSERT>(data_element_name + "_insert", std::bind(&DataElement::_insert, this, _1, _2));
        _get_list_server = node_ptr->create_service<GETLIST>(data_element_name + "_get_list", std::bind(&DataElement::_getList, this, _1, _2));
        _delete_server = node_ptr->create_service<DELETE>(data_element_name + "_delete", std::bind(&DataElement::_delete, this, _1, _2));
    }
    template <typename SELECT, typename INSERT, typename GETLIST, typename DELETE>
    void DataElement<SELECT, INSERT, GETLIST, DELETE>::_insert(const std::shared_ptr<typename INSERT::Request> request,
                                                               std::shared_ptr<typename INSERT::Response> response)
    {
        if (request)
        {
            auto result_msg = std_msgs::msg::Bool();
            try
            {
                if (_msg)
                {
                    _data_element_container.insert_or_assign(_msg.value().time_stamp.data, _msg.value());
                    result_msg.data = true;
                    response->result = result_msg;
                    _msg.reset();
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
        if (request)
        {
            try
            {
                if (response)
                {
                    *response = _data_element_container.at(request->time_stamp.data);
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(rclcpp::get_logger("debug"), "There is no data for requested timestamp");
                response = std::make_shared<typename SELECT::Response>();
            }
        }
    }
    template <typename SELECT, typename INSERT, typename GETLIST, typename DELETE>
    void DataElement<SELECT, INSERT, GETLIST, DELETE>::_insert_cb(const typename SELECT::Response::SharedPtr input_msg_ptr)
    {
        if (input_msg_ptr)
        {
            _msg = *input_msg_ptr;
        }
    }

} // end of data_store

#endif