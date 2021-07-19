#ifndef HELPERS_COMMONS__PARAMETERS_HPP_
#define HELPERS_COMMONS__PARAMETERS_HPP_

// ___CPP___
#include <random>

// ___ROS___
#include <rclcpp/rclcpp.hpp>

// ___JSON___
#include <nlohmann/json.hpp>

namespace helpers
{
  namespace commons
  {
    using json = nlohmann::json;

    /**
     * @brief Get the Parameter object
     * 
     * @tparam RepT 
     * @tparam RatioT 
     * @param parameter_name 
     * @param timeout 
     * @return json 
     */
    template <typename RepT = int64_t, typename RatioT = std::milli>
    json getParameter(const std::string &parameter_name, std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(10))
    {
      auto get_unique_node_name = []() -> std::string
      {
        static const std::string chars = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
        std::random_device rd;
        std::minstd_rand g{rd()};
        std::uniform_int_distribution<std::string::size_type> pick(0, chars.length() - 1);
        std::string s{"parameters_client_"};
        size_t orig_length = s.length();
        s.resize(orig_length + 16);
        for (size_t i = orig_length; i < s.length(); ++i)
          s[i] = chars[pick(g)];
        return s;
      };

      const std::string node_name = get_unique_node_name();
      rclcpp::Node::SharedPtr reading_parameter_node = rclcpp::Node::make_shared(node_name,
                                                                                 rclcpp::NodeOptions()
                                                                                     .use_global_arguments(false)
                                                                                     .enable_rosout(false)
                                                                                     .start_parameter_services(false)
                                                                                     .start_parameter_event_publisher(false));
      auto parameters_server_client = rclcpp::SyncParametersClient::make_shared(reading_parameter_node, "parameters_server");
      json result;
      if (!parameters_server_client->wait_for_service(timeout))
      {
        RCLCPP_DEBUG_STREAM(reading_parameter_node->get_logger(), "Parameters server is not ready while reading \"" << parameter_name << "\" parameter");
        if (!rclcpp::ok())
        {
          rclcpp::shutdown();
          throw std::runtime_error("Interrupted while waiting for the service. Exiting.");
        }
      }
      else
      {
        try
        {
          std::string parameter_str = parameters_server_client->get_parameter<std::string>(parameter_name);
          result = json::parse(parameter_str);
        }
        catch (const json::exception &e)
        {
          RCLCPP_ERROR_STREAM(reading_parameter_node->get_logger(), "Reading \"" << parameter_name << "\" JSON error: " << e.what());
        }
        catch (const std::exception &e)
        {
          RCLCPP_ERROR_STREAM(reading_parameter_node->get_logger(), "Reading \"" << parameter_name << "\" error: " << e.what());
        }
      }
      parameters_server_client.reset();
      reading_parameter_node.reset();
      return result;
    }

    /**
     * @brief Get the Parameters object
     * 
     * @tparam RepT 
     * @tparam RatioT 
     * @param parameters_names 
     * @param timeout 
     * @return std::map<std::string, json> 
     */
    template <typename RepT = int64_t, typename RatioT = std::milli>
    std::map<std::string, json> getParameters(const std::vector<std::string> &parameters_names, std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(10))
    {
      auto get_unique_node_name = []() -> std::string
      {
        static const std::string chars = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
        std::random_device rd;
        std::minstd_rand g{rd()};
        std::uniform_int_distribution<std::string::size_type> pick(0, chars.length() - 1);
        std::string s{"parameters_client_"};
        size_t orig_length = s.length();
        s.resize(orig_length + 16);
        for (size_t i = orig_length; i < s.length(); ++i)
          s[i] = chars[pick(g)];
        return s;
      };

      const std::string node_name = get_unique_node_name();
      rclcpp::Node::SharedPtr reading_parameter_node = rclcpp::Node::make_shared(node_name,
                                                                                 rclcpp::NodeOptions()
                                                                                     .use_global_arguments(false)
                                                                                     .enable_rosout(false)
                                                                                     .start_parameter_services(false)
                                                                                     .start_parameter_event_publisher(false));
      auto parameters_server_client = rclcpp::SyncParametersClient::make_shared(reading_parameter_node, "parameters_server");
      std::map<std::string, json> result;
      result.clear();
      if (!parameters_server_client->wait_for_service(timeout))
      {
        RCLCPP_DEBUG(reading_parameter_node->get_logger(), "Parameters server is not ready");
        if (!rclcpp::ok())
        {
          rclcpp::shutdown();
          throw std::runtime_error("Interrupted while waiting for the service. Exiting.");
        }
      }
      else
      {
        for (const auto &parameter_name : parameters_names)
        {
          try
          {
            std::string parameter_str = parameters_server_client->get_parameter<std::string>(parameter_name);
            result[parameter_name] = json::parse(parameter_str);
          }
          catch (const json::exception &e)
          {
            RCLCPP_ERROR_STREAM(reading_parameter_node->get_logger(), "Reading \"" << parameter_name << "\" JSON error: " << e.what());
            result.clear();
            break;
          }
          catch (const std::exception &e)
          {
            RCLCPP_ERROR_STREAM(reading_parameter_node->get_logger(), "Reading \"" << parameter_name << "\" error: " << e.what());
            result.clear();
            break;
          }
        }
      }
      parameters_server_client.reset();
      reading_parameter_node.reset();
      return result;
    }
  } // namespace commons
} // namespace helpers

#endif // HELPERS_COMMONS__PARAMETERS_HPP_
