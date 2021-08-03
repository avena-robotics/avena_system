#ifndef ESTIMATE_SHAPE_LOGGING_MACROS_HPP
#define ESTIMATE_SHAPE_LOGGING_MACROS_HPP

#include <rclcpp/rclcpp.hpp>

namespace estimate_shape
{
  static const rclcpp::Logger LOGGER = rclcpp::get_logger("estimate_shape");

// DEBUG
#define LOG_DEBUG(...)                 \
  do                                   \
  {                                    \
    RCLCPP_DEBUG(LOGGER, __VA_ARGS__); \
  } while (0)

#define LOG_DEBUG_STREAM(stream_arg)         \
  do                                         \
  {                                          \
    RCLCPP_DEBUG_STREAM(LOGGER, stream_arg); \
  } while (0)

// INFO
#define LOG_INFO(...)                 \
  do                                  \
  {                                   \
    RCLCPP_INFO(LOGGER, __VA_ARGS__); \
  } while (0)

#define LOG_INFO_STREAM(stream_arg)         \
  do                                        \
  {                                         \
    RCLCPP_INFO_STREAM(LOGGER, stream_arg); \
  } while (0)

// WARN
#define LOG_WARN(...)                 \
  do                                  \
  {                                   \
    RCLCPP_WARN(LOGGER, __VA_ARGS__); \
  } while (0)

#define LOG_WARN_STREAM(stream_arg)         \
  do                                        \
  {                                         \
    RCLCPP_WARN_STREAM(LOGGER, stream_arg); \
  } while (0)

// ERROR
#define LOG_ERROR(...)                 \
  do                                   \
  {                                    \
    RCLCPP_ERROR(LOGGER, __VA_ARGS__); \
  } while (0)

#define LOG_ERROR_STREAM(stream_arg)         \
  do                                         \
  {                                          \
    RCLCPP_ERROR_STREAM(LOGGER, stream_arg); \
  } while (0)

// FATAL
#define LOG_FATAL(...)                 \
  do                                   \
  {                                    \
    RCLCPP_FATAL(LOGGER, __VA_ARGS__); \
  } while (0)

#define LOG_FATAL_STREAM(stream_arg)         \
  do                                         \
  {                                          \
    RCLCPP_FATAL_STREAM(LOGGER, stream_arg); \
  } while (0)

} // namespace estimate_shape

#endif // ESTIMATE_SHAPE_LOGGING_MACROS_HPP
