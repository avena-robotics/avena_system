#include "data_store/data_store.hpp"
namespace data_store
{
    DataStore::DataStore(const rclcpp::NodeOptions &options) : Node("DataStore", options)
    {
        status = custom_interfaces::msg::Heartbeat::STOPPED;
        RCLCPP_INFO(this->get_logger(), "started Node");
        _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
        using namespace std::placeholders;
        rclcpp::QoS qos_latching = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        auto _rgb_data_callback = [this](const RgbDataSelect::Response::SharedPtr input_msg_ptr) -> void
        {
            if (input_msg_ptr)
            {
                _rgb_data_msg = *input_msg_ptr;
            }
        };
        auto _item_cam1_callback = [this](const ItemCam1Select::Response::SharedPtr input_msg_ptr) -> void
        {
            if (input_msg_ptr)
            {
                _item_cam1_msg = *input_msg_ptr;
            }
        };
        auto _item_cam2_callback = [this](const ItemCam2Select::Response::SharedPtr input_msg_ptr) -> void
        {
            if (input_msg_ptr)
            {
                _item_cam2_msg = *input_msg_ptr;
            }
        };

        _rgb_data_insert_sub = create_subscription<RgbDataSelect::Response>("rgb_data", qos_latching, _rgb_data_callback);
        _item_cam_1_insert_sub = create_subscription<ItemCam1Select::Response>("item_cam1", qos_latching, _item_cam1_callback);
        _item_cam_2_insert_sub = create_subscription<ItemCam2Select::Response>("item_cam2", qos_latching, _item_cam2_callback);

        _rgb_data_insert_server = create_service<RgbDataInsert>("rgb_data_insert", std::bind(&DataStore::_insertRgbData, this, _1, _2));
        _rgb_data_delete_server = create_service<RgbDataDelete>("rgb_data_delete", std::bind(&DataStore::_deleteRgbData, this, _1, _2));
        _rgb_data_select_server = create_service<RgbDataSelect>("rgb_data_select", std::bind(&DataStore::_selectRgbData, this, _1, _2));
        _rgb_data_get_list_server = create_service<RgbDataGetList>("rgb_data_get_list", std::bind(&DataStore::_getListRgbData, this, _1, _2));

        _item_cam_1_insert_server = create_service<ItemCam1Insert>("item_cam_1_insert", std::bind(&DataStore::_insertItemCam1, this, _1, _2));
        _item_cam_1_delete_server = create_service<ItemCam1Delete>("item_cam_1_delete", std::bind(&DataStore::_deleteItemCam1, this, _1, _2));
        _item_cam_1_select_server = create_service<ItemCam1Select>("item_cam_1_select", std::bind(&DataStore::_selectItemCam1, this, _1, _2));
        _item_cam_1_get_list_server = create_service<ItemCam1GetList>("item_cam_1_get_list", std::bind(&DataStore::_getListItemCam1, this, _1, _2));

        _item_cam_2_insert_server = create_service<ItemCam2Insert>("item_cam_2_insert", std::bind(&DataStore::_insertItemCam2, this, _1, _2));
        _item_cam_2_delete_server = create_service<ItemCam2Delete>("item_cam_2_delete", std::bind(&DataStore::_deleteItemCam2, this, _1, _2));
        _item_cam_2_select_server = create_service<ItemCam2Select>("item_cam_2_select", std::bind(&DataStore::_selectItemCam2, this, _1, _2));
        _item_cam_2_get_list_server = create_service<ItemCam2GetList>("item_cam_2_get_list", std::bind(&DataStore::_getListItemCam2, this, _1, _2));

        // _rgb_data.clear();
        // _item_cam1.clear();
        // _item_cam2.clear();
        signal(SIGINT, _signalHandler);
    }
    void DataStore::_signalHandler(int signum)
    {
        (void)signum;
        rclcpp::shutdown();
        // supposed to be signum
        exit(0);
    }

    void DataStore::initNode()
    {
        status = custom_interfaces::msg::Heartbeat::STARTING;
        RCLCPP_INFO(get_logger(), "Initialization of data store.");
        status = custom_interfaces::msg::Heartbeat::RUNNING;
    }

    void DataStore::shutDownNode()
    {
        RCLCPP_INFO(this->get_logger(), "shut Down Node");
        if (status != custom_interfaces::msg::Heartbeat::STOPPED)
            status = custom_interfaces::msg::Heartbeat::STOPPED;
    }
    DataStore::~DataStore()
    {
        shutDownNode();
    }
    void DataStore::_insertRgbData(const std::shared_ptr<RgbDataInsert::Request> request,
                                   std::shared_ptr<RgbDataInsert::Response> response)
    {
        if (request)
        {
            auto result_msg = std_msgs::msg::Bool();
            try
            {
                if (_rgb_data_msg)
                {
                    _rgb_data.insert_or_assign(_rgb_data_msg.value().time_stamp.data, _rgb_data_msg.value());
                    result_msg.data = true;
                    response->result = result_msg;
                    // for (const auto &[_, rgb_data_element_value] : _rgb_data)
                    // {
                    //     RCLCPP_INFO(this->get_logger(), std::to_string(rgb_data_element_value.time_stamp.data));
                    //     RCLCPP_INFO(this->get_logger(), std::to_string(rgb_data_element_value.camera_1_rgb.height));
                    //     RCLCPP_INFO(this->get_logger(), std::to_string(rgb_data_element_value.camera_2_rgb.height));
                    //     RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------");
                    // }
                    _rgb_data_msg.reset();
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "No data on topic rgb_data");
                    result_msg.data = false;
                    response->result = result_msg;
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_logger(), e.what());
                result_msg.data = false;
                response->result = result_msg;
            }
        }
    }
    void DataStore::_insertItemCam1(const std::shared_ptr<ItemCam1Insert::Request> request,
                                    std::shared_ptr<ItemCam1Insert::Response> response)
    {
        if (request)
        {
            auto result_msg = std_msgs::msg::Bool();
            try
            {
                if (_item_cam1_msg)
                {
                    _item_cam1.insert_or_assign(_item_cam1_msg.value().time_stamp.data, _item_cam1_msg.value());
                    result_msg.data = true;
                    response->result = result_msg;
                    // for (const auto &[_, _item_cam1_element_value] : _item_cam1)
                    // {
                    //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam1_element_value.time_stamp.data));
                    //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam1_element_value.item_cam1_labels.size()));
                    //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam1_element_value.item_cam1_masks.size()));
                    //     RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------");
                    // }
                    _item_cam1_msg.reset();
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "No data on topic item_cam1");
                    result_msg.data = false;
                    response->result = result_msg;
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_logger(), e.what());
                result_msg.data = false;
                response->result = result_msg;
            }
        }
    }
    void DataStore::_insertItemCam2(const std::shared_ptr<ItemCam2Insert::Request> request,
                                    std::shared_ptr<ItemCam2Insert::Response> response)
    {
        if (request)
        {
            auto result_msg = std_msgs::msg::Bool();
            try
            {
                if (_item_cam2_msg)
                {
                    _item_cam2.insert_or_assign(_item_cam2_msg.value().time_stamp.data,_item_cam2_msg.value());
                    result_msg.data = true;
                    response->result = result_msg;
                    // for (const auto &[_, _item_cam2_element_value] : _item_cam2)
                    // {
                    //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam2_element_value.time_stamp.data));
                    //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam2_element_value.item_cam2_labels.size()));
                    //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam2_element_value.item_cam2_masks.size()));
                    //     RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------");
                    // }
                    _item_cam2_msg.reset();
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "No data on topic item_cam2");
                    result_msg.data = false;
                    response->result = result_msg;
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_logger(), e.what());
                result_msg.data = false;
                response->result = result_msg;
            }
        }
    }
    void DataStore::_deleteRgbData(const std::shared_ptr<RgbDataDelete::Request> request,
                                   std::shared_ptr<RgbDataDelete::Response> response)
    {
        if (request)
        {
            auto result_msg = std_msgs::msg::Bool();

            if (_rgb_data.erase(request->time_stamp.data) != 0)
            {
                result_msg.data = true;
                response->result = result_msg;
                // for (const auto &[_, rgb_data_element_value] : _rgb_data)
                // {
                //     RCLCPP_INFO(this->get_logger(), std::to_string(rgb_data_element_value.time_stamp.data));
                //     RCLCPP_INFO(this->get_logger(), std::to_string(rgb_data_element_value.camera_1_rgb.height));
                //     RCLCPP_INFO(this->get_logger(), std::to_string(rgb_data_element_value.camera_2_rgb.height));
                //     RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------");
                // }
            }

            else
            {
                RCLCPP_WARN(get_logger(), "item is not found, you can not delete it");
                result_msg.data = false;
                response->result = result_msg;
            }
        }
    }

    void DataStore::_deleteItemCam1(const std::shared_ptr<ItemCam1Delete::Request> request,
                                    std::shared_ptr<ItemCam1Delete::Response> response)
    {
        if (request)
        {
            auto result_msg = std_msgs::msg::Bool();

            if (_item_cam1.erase(request->time_stamp.data) != 0)
            {
                result_msg.data = true;
                response->result = result_msg;
                // for (const auto &[_, _item_cam1_element_value] : _item_cam1)
                // {
                //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam1_element_value.time_stamp.data));
                //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam1_element_value.item_cam1_labels.size()));
                //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam1_element_value.item_cam1_masks.size()));
                //     RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------");
                // }
            }

            else
            {
                RCLCPP_WARN(get_logger(), "item is not found, you can not delete it");
                result_msg.data = false;
                response->result = result_msg;
            }
        }
    }
    void DataStore::_deleteItemCam2(const std::shared_ptr<ItemCam2Delete::Request> request,
                                    std::shared_ptr<ItemCam2Delete::Response> response)
    {
        if (request)
        {
            auto result_msg = std_msgs::msg::Bool();

            if (_item_cam2.erase(request->time_stamp.data) != 0)
            {
                result_msg.data = true;
                response->result = result_msg;
                // for (const auto &[_, _item_cam2_element_value] : _item_cam2)
                // {
                //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam2_element_value.time_stamp.data));
                //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam2_element_value.item_cam2_labels.size()));
                //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam2_element_value.item_cam2_masks.size()));
                //     RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------");
                // }
            }

            else
            {
                RCLCPP_WARN(get_logger(), "item is not found, you can not delete it");
                result_msg.data = false;
                response->result = result_msg;
            }
        }
    }
    void DataStore::_selectRgbData(const std::shared_ptr<RgbDataSelect::Request> request,
                                   std::shared_ptr<RgbDataSelect::Response> response)
    {
        if (request)
        {
            try
            {
                response->time_stamp = request->time_stamp;
                response->camera_1_rgb = _rgb_data.at(request->time_stamp.data).camera_1_rgb;
                response->camera_2_rgb = _rgb_data.at(request->time_stamp.data).camera_2_rgb;
                // for (const auto &[_, rgb_data_element_value] : _rgb_data)
                // {
                //     RCLCPP_INFO(this->get_logger(), std::to_string(rgb_data_element_value.time_stamp.data));
                //     RCLCPP_INFO(this->get_logger(), std::to_string(rgb_data_element_value.camera_1_rgb.height));
                //     RCLCPP_INFO(this->get_logger(), std::to_string(rgb_data_element_value.camera_2_rgb.height));
                //     RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------");
                // }
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(get_logger(), "There is no data for requested timestamp");
                response = RgbDataSelect::Response::SharedPtr();
            }
        }
    }
    void DataStore::_selectItemCam1(const std::shared_ptr<ItemCam1Select::Request> request,
                                    std::shared_ptr<ItemCam1Select::Response> response)
    {
        if (request)
        {
            try
            {
                response->time_stamp = request->time_stamp;
                response->item_cam1_labels = _item_cam1.at(request->time_stamp.data).item_cam1_labels;
                response->item_cam1_masks = _item_cam1.at(request->time_stamp.data).item_cam1_masks;
                // for (const auto &[_, _item_cam1_element_value] : _item_cam1)
                // {
                //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam1_element_value.time_stamp.data));
                //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam1_element_value.item_cam1_labels.size()));
                //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam1_element_value.item_cam1_masks.size()));
                //     RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------");
                // }
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(get_logger(), "There is no data for requested timestamp");
                response = ItemCam1Select::Response::SharedPtr();
            }
        }
    }
    void DataStore::_selectItemCam2(const std::shared_ptr<ItemCam2Select::Request> request,
                                    std::shared_ptr<ItemCam2Select::Response> response)
    {
        if (request)
        {
            try
            {
                response->time_stamp = request->time_stamp;
                response->item_cam2_labels = _item_cam2.at(request->time_stamp.data).item_cam2_labels;
                response->item_cam2_masks = _item_cam2.at(request->time_stamp.data).item_cam2_masks;
                // for (const auto &[_, _item_cam2_element_value] : _item_cam2)
                // {
                //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam2_element_value.time_stamp.data));
                //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam2_element_value.item_cam2_labels.size()));
                //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam2_element_value.item_cam2_masks.size()));
                //     RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------");
                // }
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(get_logger(), "There is no data for requested timestamp");
                response = ItemCam2Select::Response::SharedPtr();
            }
        }
    }
    void DataStore::_getListRgbData(const std::shared_ptr<RgbDataGetList::Request> request,
                                    std::shared_ptr<RgbDataGetList::Response> response)
    {
        if (request)
        {
            try
            {
                std_msgs::msg::Float64 timestamp_msg;
                for (const auto &[time_stamp, _] : _rgb_data)
                {
                    timestamp_msg.data = time_stamp;
                    response->time_stamps.emplace_back(timestamp_msg);
                }
                // for (const auto &[_, rgb_data_element_value] : _rgb_data)
                // {
                //     RCLCPP_INFO(this->get_logger(), std::to_string(rgb_data_element_value.time_stamp.data));
                //     RCLCPP_INFO(this->get_logger(), std::to_string(rgb_data_element_value.camera_1_rgb.height));
                //     RCLCPP_INFO(this->get_logger(), std::to_string(rgb_data_element_value.camera_2_rgb.height));
                //     RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------");
                // }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_logger(), e.what());
                response = RgbDataGetList::Response::SharedPtr();
            }
        }
    }
    void DataStore::_getListItemCam1(const std::shared_ptr<ItemCam1GetList::Request> request,
                                     std::shared_ptr<ItemCam1GetList::Response> response)
    {
        if (request)
        {
            try
            {
                std_msgs::msg::Float64 timestamp_msg;
                for (const auto &[time_stamp, _] : _item_cam1)
                {
                    timestamp_msg.data = time_stamp;
                    response->time_stamps.emplace_back(timestamp_msg);
                }
                // for (const auto &[_, _item_cam1_element_value] : _item_cam1)
                // {
                //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam1_element_value.time_stamp.data));
                //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam1_element_value.item_cam1_labels.size()));
                //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam1_element_value.item_cam1_masks.size()));
                //     RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------");
                // }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_logger(), e.what());
                response = ItemCam1GetList::Response::SharedPtr();
            }
        }
    }
    void DataStore::_getListItemCam2(const std::shared_ptr<ItemCam2GetList::Request> request,
                                     std::shared_ptr<ItemCam2GetList::Response> response)
    {
        if (request)
        {
            try
            {
                std_msgs::msg::Float64 timestamp_msg;
                for (const auto &[time_stamp, _] : _item_cam2)
                {
                    timestamp_msg.data = time_stamp;
                    response->time_stamps.emplace_back(timestamp_msg);
                }
                // for (const auto &[_, _item_cam2_element_value] : _item_cam2)
                // {
                //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam2_element_value.time_stamp.data));
                //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam2_element_value.item_cam2_labels.size()));
                //     RCLCPP_INFO(this->get_logger(), std::to_string(_item_cam2_element_value.item_cam2_masks.size()));
                //     RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------");
                // }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_logger(), e.what());
                response = ItemCam2GetList::Response::SharedPtr();
            }
        }
    }
    DataState_e DataStore::_validateInput()
    {
        // if (!_input_msg_data)
        // {
        //   RCLCPP_WARN(get_logger(), "Module has not received data yet.");
        //   return DataState_e::EMPTY;
        // }
        // if (_input_msg_data->header.stamp == builtin_interfaces::msg::Time())
        // {
        //   RCLCPP_WARN(get_logger(), "Invalid input message header.");
        //   return DataState_e::INVALID_HEADER;
        // }
        return DataState_e::VALID;
    }
    DataState_e DataStore::_validateOutput()
    {
        // if (!_input_msg_data)
        // {
        //   RCLCPP_WARN(get_logger(), "Module has not received data yet.");
        //   return 1;
        // }
        // if (_input_msg_data->header.stamp == builtin_interfaces::msg::Time())
        // {
        //   RCLCPP_WARN(get_logger(), "Invalid input message header.");
        //   return 1;
        // }
        return DataState_e::VALID;
    }

} // end of DataStore_bt
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(data_store::DataStore)
