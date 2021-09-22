#include "ptcld_merger/ptcld_merger.hpp"
namespace ptcld_merger
{
    PtcldMerger::PtcldMerger(const rclcpp::NodeOptions &options) : Node("ptcld_merger", options)
    {
        status = custom_interfaces::msg::Heartbeat::STOPPED;
        RCLCPP_INFO(this->get_logger(), " node ptcld_merger started");
        _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
    }
    void PtcldMerger::initNode()
    {
        status = custom_interfaces::msg::Heartbeat::STARTING;
        RCLCPP_INFO(get_logger(), "Initialization of ptcld_merger");
        status = custom_interfaces::msg::Heartbeat::RUNNING;
    }
    void PtcldMerger::shutDownNode()
    {
        RCLCPP_INFO(this->get_logger(), "node ptcld_merger is shut Down");
        if (status != custom_interfaces::msg::Heartbeat::STOPPED)
            status = custom_interfaces::msg::Heartbeat::STOPPED;
    }
    // void PtcldMerger::_convertCloudToPCL(const sensor_msgs::msg::PointCloud2 &ros_cloud_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr &out_pcl_cloud)
    // {
    //     helpers::converters::rosPtcldtoPcl<pcl::PointXYZ>(ros_cloud_msg, out_pcl_cloud);
    // }

    // void PtcldMerger::_convertCloudToRos(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_cloud, sensor_msgs::msg::PointCloud2 &out_ros_cloud_msg)
    // {
    //     helpers::converters::pclToRosPtcld<pcl::PointXYZ>(pcl_cloud, out_ros_cloud_msg);
    // }

} // namespace ptcld_merger
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ptcld_merger::PtcldMerger)