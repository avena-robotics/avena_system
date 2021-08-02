#include "data_store/data_store.hpp"
namespace data_store
{
    DataStore::DataStore(const rclcpp::NodeOptions &options) : Node("DataStore", options)
    {
        status = custom_interfaces::msg::Heartbeat::STOPPED;
        RCLCPP_INFO(this->get_logger(), "started DataStore Node");
        _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
    }

    void DataStore::initNode()
    {
        status = custom_interfaces::msg::Heartbeat::STARTING;
        RCLCPP_INFO(get_logger(), "Initialization of data store.");
        // rclcpp::QoS qos_latching = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        // _tracker_element_ptr = std::make_unique<Tracker>(shared_from_this(), qos_latching, "tracker");
        // _tracker_element_ptr = std::make_unique<Tracker>(shared_from_this(), "tracker");
        _cameras_data_element_ptr = std::make_unique<RgbdSync>(shared_from_this(), "rgbd_sync");
        _detectron_element_ptr = std::make_unique<Detectron>(shared_from_this(), "filter_detections");
        _items_element_ptr = std::make_unique<Items>(shared_from_this(), "compose_items");
        // add other data elements here
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

} // end of DataStore_bt
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(data_store::DataStore)
