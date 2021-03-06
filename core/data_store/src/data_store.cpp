#include "data_store/data_store.hpp"
namespace data_store
{
    DataStore::DataStore(const rclcpp::NodeOptions &options) : Node("data_store", options)
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
        _tracker_element_ptr = std::make_unique<Tracker>(shared_from_this(), "tracker");
        _cameras_data_element_ptr = std::make_unique<RgbdSync>(shared_from_this(), "rgbd_sync");
        _detectron_element_ptr = std::make_unique<Detectron>(shared_from_this(), "detectron");
        _items_element_ptr = std::make_unique<Items>(shared_from_this(), "items");
        _scene_element_ptr = std::make_unique<Scene>(shared_from_this(), "scene");
        _movement_sequence_element_ptr = std::make_unique<MovementSequence>(shared_from_this(), "movement_sequence");
        _trajectory_element_ptr = std::make_unique<Trajectory>(shared_from_this(), "trajectory");
        // add other data elements here
        status = custom_interfaces::msg::Heartbeat::RUNNING;
    }

    void DataStore::shutDownNode()
    {
        RCLCPP_INFO(this->get_logger(), "shut Down Node");
        if (status != custom_interfaces::msg::Heartbeat::STOPPED)
            status = custom_interfaces::msg::Heartbeat::STOPPED;
        _tracker_element_ptr.reset();            
        _cameras_data_element_ptr.reset();            
        _detectron_element_ptr.reset();            
        _items_element_ptr.reset();            
        _scene_element_ptr.reset();
        _movement_sequence_element_ptr.reset();
        _trajectory_element_ptr.reset();
        // add other data elements here            
    }
    DataStore::~DataStore()
    {
        shutDownNode();
    }

} // end of DataStore_bt
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(data_store::DataStore)
