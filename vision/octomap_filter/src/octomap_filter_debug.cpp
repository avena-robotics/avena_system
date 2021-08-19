#include "octomap_filter_debug.hpp"

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/parameter.hpp>

namespace ros2mysql
{

    OctomapFilter::OctomapFilter(const rclcpp::NodeOptions &options)
        : Node("octomap_filter_debug", options)
    {
        // Declare parameters.
        this->initialize_parameters();

        this->configure();

        using namespace std::chrono_literals;
        this->db_ = std::make_unique<MysqlConnector>(this->host_, this->port_, this->db_name_, this->username_, this->password_, this->debug_);
        auto callback =
            [this](const typename custom_interfaces::msg::FilteredSceneOctomap::SharedPtr msg) -> void {
            RCLCPP_INFO(this->get_logger(), "OctomapFilter debug: received data");
             std::vector<uint32_t> scene_ids = db_->getRowsId("scene");
            scene_t scene_data;
            if (scene_ids.size() > 0)
                scene_data.scene_id = scene_ids.back();

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            helpers::converters::rosPtcldtoPcl<pcl::PointXYZ>(msg->filtered_scene_octomap, cloud);
            helpers::converters::pointcloudToStream<pcl::PointXYZ>(cloud, scene_data.filtered_octomap);
            db_->setScene(&scene_data);
            cloud->clear();
            RCLCPP_INFO(this->get_logger(), "OctomapFilter debug: saved");
        };

        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(1));//.transient_local().reliable();
        sub_ = create_subscription<custom_interfaces::msg::FilteredSceneOctomap>("octomap_filter", qos_settings, callback);
    }

    void
    OctomapFilter::configure()
    {
        this->get_parameter<std::string>("host", host_);
        this->get_parameter<std::string>("port", port_);
        this->get_parameter<std::string>("db_name", db_name_);
        this->get_parameter<std::string>("username", username_);
        this->get_parameter<std::string>("password", password_);
        this->get_parameter<bool>("debug", debug_);
    }

    void OctomapFilter::initialize_parameters()
    {
        rcl_interfaces::msg::ParameterDescriptor host_descriptor;
        host_descriptor.name = "host";
        host_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        this->declare_parameter("host", "", host_descriptor);

        rcl_interfaces::msg::ParameterDescriptor port_descriptor;
        port_descriptor.name = "port";
        port_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        this->declare_parameter("port", "", port_descriptor);

        rcl_interfaces::msg::ParameterDescriptor db_name_descriptor;
        db_name_descriptor.name = "db_name";
        db_name_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        this->declare_parameter("db_name", "", db_name_descriptor);

        rcl_interfaces::msg::ParameterDescriptor username_descriptor;
        username_descriptor.name = "username";
        username_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        this->declare_parameter("username", "", username_descriptor);

        rcl_interfaces::msg::ParameterDescriptor password_descriptor;
        password_descriptor.name = "password";
        password_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        this->declare_parameter("password", "", password_descriptor);

        rcl_interfaces::msg::ParameterDescriptor debug_descriptor;
        debug_descriptor.name = "debug";
        debug_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        this->declare_parameter("debug", false, debug_descriptor);
    }

} // namespace ros2mysql

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ros2mysql::OctomapFilter)
