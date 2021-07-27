#include "data_store/data_store.hpp"
namespace data_store
{
    DataStore::DataStore(const rclcpp::NodeOptions &options) : Node("DataStore", options)
    {
        status = custom_interfaces::msg::Heartbeat::STOPPED;
        RCLCPP_INFO(this->get_logger(), "started Node");
        _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
    }

    void DataStore::initNode()
    {
        status = custom_interfaces::msg::Heartbeat::STARTING;
        RCLCPP_INFO(get_logger(), "Initialization of data store.");
        rclcpp::QoS qos_latching = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        _rgb_data_element_ptr = std::make_unique<RgbData>(shared_from_this(), qos_latching, "rgb_data");
        _item_cam1_element_ptr = std::make_unique<ItemCam1>(shared_from_this(), qos_latching, "item_cam1");
        _item_cam2_element_ptr = std::make_unique<ItemCam2>(shared_from_this(), qos_latching, "item_cam2");
        _tracker_element_ptr = std::make_unique<Tracker>(shared_from_this(), qos_latching, "tracker");
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
