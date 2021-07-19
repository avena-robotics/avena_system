#ifndef HELPERS_COMMONS__LOGGING_HPP_
#define HELPERS_COMMONS__LOGGING_HPP_

// ___CPP___
#include <memory>
#include <chrono>
#include <iostream>
#include <cstring>

// ___ROS___
#include <rclcpp/rclcpp.hpp>

namespace helpers
{
  namespace commons
  {
    static const rclcpp::Logger HELPERS_LOGGER = rclcpp::get_logger("helpers");

    /**
     * @brief Set the Logger Level object
     * 
     * @param logger_name 
     * @param logger_level 
     * @return int 
     */
    int setLoggerLevel(std::string logger_name, std::string logger_level);

    /**
     * @brief Set the Logger Level object
     * 
     * @param logger 
     * @param logger_level 
     * @return int 
     */
    int setLoggerLevel(rclcpp::Logger logger, std::string logger_level);

    /**
     * @brief Set the Logger Level From Parameter object
     * 
     * @param node 
     * @return int 
     */
    int setLoggerLevelFromParameter(rclcpp::Node *node);

    /**
     * @brief Set the Logger Level From Parameter object
     * 
     * @param node 
     * @return int 
     */
    int setLoggerLevelFromParameter(rclcpp::Node::SharedPtr node);

    class Logger
    {
    public:
      explicit Logger(bool debug = false)
          : _debug(debug) {}

      ~Logger() = default;

      template <typename... Args>
      void log(Args &&...args)
      {
        if (_debug)
          log_(std::forward<Args>(args)...);
      }

    private:
      bool _debug;

      template <typename Head>
      void log_(Head &&head)
      {
        std::cout << std::forward<Head>(head) << '\n';
      }

      template <typename Head, typename... Tail>
      void log_(Head &&head, Tail &&...tail)
      {
        std::cout << std::forward<Head>(head) << ' ';
        log_(std::forward<Tail>(tail)...);
      }
    };
  } // // namespace commons
} // namespace helpers

#endif // HELPERS_COMMONS__LOGGING_HPP_
