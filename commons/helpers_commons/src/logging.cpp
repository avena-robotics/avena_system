#include "helpers_commons/logging.hpp"

namespace helpers
{
    namespace commons
    {
        int setLoggerLevel(std::string logger_name, std::string logger_level)
        {
            std::transform(logger_level.begin(), logger_level.end(), logger_level.begin(), [](unsigned char c) -> unsigned char
                           { return std::tolower(c); });

            int logger_severity_level;
            if (logger_level == "debug")
                logger_severity_level = RCUTILS_LOG_SEVERITY_DEBUG;
            else if (logger_level == "info")
                logger_severity_level = RCUTILS_LOG_SEVERITY_INFO;
            else if (logger_level == "warn")
                logger_severity_level = RCUTILS_LOG_SEVERITY_WARN;
            else if (logger_level == "error")
                logger_severity_level = RCUTILS_LOG_SEVERITY_ERROR;
            else if (logger_level == "fatal")
                logger_severity_level = RCUTILS_LOG_SEVERITY_FATAL;
            else
            {
                RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "Invalid severity level for \"" << logger_name << "\" logger. Setting to INFO.");
                logger_severity_level = RCUTILS_LOG_SEVERITY_INFO;
            }
            auto ret = rcutils_logging_set_logger_level(logger_name.c_str(), logger_severity_level);
            if (ret != RCUTILS_RET_OK)
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name), "Error setting severity: " << rcutils_get_error_string().str);
                rcutils_reset_error();
                return 1;
            }
            return 0;
        }

        int setLoggerLevel(rclcpp::Logger logger, std::string logger_level)
        {
            return setLoggerLevel(logger.get_name(), logger_level);
        }

        int setLoggerLevelFromParameter(rclcpp::Node *node)
        {
            if (!node)
                return 1;
            rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
            const std::string log_level_param_name = "log_level";
            parameter_descriptor.name = log_level_param_name;
            parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            parameter_descriptor.read_only = false;
            node->declare_parameter(log_level_param_name, "info", parameter_descriptor);
            std::string log_level;
            node->get_parameter<std::string>(log_level_param_name, log_level);
            return setLoggerLevel(node->get_logger(), log_level);
        }

        int setLoggerLevelFromParameter(rclcpp::Node::SharedPtr node)
        {
            return setLoggerLevelFromParameter(node.get());
        }
    } // namespace commons
} // namespace helpers
