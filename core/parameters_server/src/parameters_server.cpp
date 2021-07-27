#include <memory>
#include <string>
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

namespace parameters_server
{
    class ParametersServer : public rclcpp::Node
    {
    public:
        explicit ParametersServer(rclcpp::NodeOptions options)
            : Node("parameters_server",
                   options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
        {
            rcl_interfaces::msg::ListParametersResult parameters = list_parameters({}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);
            RCLCPP_INFO_STREAM(get_logger(), "Parameter blackboard node named \""
                                                 << get_fully_qualified_name() << "\" ready, and serving "
                                                 << parameters.names.size() << " parameters already!");
        }
    };

} // namespace parameters_server
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(parameters_server::ParametersServer)
