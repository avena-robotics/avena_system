#include "visualization_tools/scene_visualization.hpp"

namespace visualization_tools
{
    SceneVisualization::SceneVisualization(const rclcpp::NodeOptions &options)
        : Node("scene_visualization", options)
    {
        helpers::commons::setLoggerLevel(get_logger(), "debug");
        _getParametersFromServer();
        _initializeMarkersPublishers();
        _static_markers_timer = create_wall_timer(std::chrono::seconds(1), std::bind(&SceneVisualization::_publishStaticMarkers, this));
    }

    int SceneVisualization::_getParametersFromServer()
    {
        RCLCPP_INFO_ONCE(get_logger(), "Reading parameters from the server");
        while (true)
        {
            auto parameters = helpers::commons::getParameters({"areas"});
            if (parameters.empty())
                continue;

            _areas_parameters = parameters["areas"];
            break;
        }

        RCLCPP_INFO(get_logger(), "Parameters read successfully...");
        return 0;
    }

    int SceneVisualization::_initializeMarkersPublishers()
    {
        _table_marker_pub = create_publisher<visualization_msgs::msg::Marker>("scene_markers", 1);
        return 0;
    }

    int SceneVisualization::_publishStaticMarkers()
    {
        // Table area
        float table_x_min = _areas_parameters["table_area"]["min"]["x"].get<float>();
        float table_y_min = _areas_parameters["table_area"]["min"]["y"].get<float>();
        // float table_z_min = _areas_parameters["table_area"]["min"]["z"].get<float>();
        float table_x_max = _areas_parameters["table_area"]["max"]["x"].get<float>();
        float table_y_max = _areas_parameters["table_area"]["max"]["y"].get<float>();
        // float table_z_max = _areas_parameters["table_area"]["max"]["z"].get<float>();

        const float table_thickness = 0.01; // it does not matter how thick the table is, so set this value to 1 cm
        visualization_msgs::msg::Marker table_area_marker;
        table_area_marker.header.frame_id = "world";
        table_area_marker.ns = "scene";
        table_area_marker.id = 0;
        table_area_marker.type = visualization_msgs::msg::Marker::CUBE;
        table_area_marker.action = visualization_msgs::msg::Marker::ADD;
        table_area_marker.pose.position.x = (table_x_max + table_x_min) / 2;
        table_area_marker.pose.position.y = (table_y_max + table_y_min) / 2;
        table_area_marker.pose.position.z = -table_thickness / 2;
        table_area_marker.scale.x = table_x_max - table_x_min;
        table_area_marker.scale.y = table_y_max - table_y_min;
        table_area_marker.scale.z = table_thickness;  
        table_area_marker.color.r = 1.0;
        table_area_marker.color.g = 0.61;
        table_area_marker.color.b = 0.48;
        table_area_marker.color.a = 1.0;
        _table_marker_pub->publish(table_area_marker);
        return 0;
    }

} // namespace visualization_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(visualization_tools::SceneVisualization)
