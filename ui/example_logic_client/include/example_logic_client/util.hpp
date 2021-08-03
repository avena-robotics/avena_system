#ifndef CLI__UTIL_HPP_
#define CLI__UTIL_HPP_

// ___CPP___
#include <chrono>

// ___ROS2___
#include "rclcpp/rclcpp.hpp"

// ___Avena___
// #include "cli/visibility_control.h"


namespace cli
{
  class Timer
  {
  public:
    // CLI_PUBLIC
    Timer(std::string name = "") : _name(name)
    {
      _start = _tic();
    }

    // CLI_PUBLIC
    ~Timer()
    {
      auto stop = _tic();
      auto duration = std::chrono::duration<double, std::milli>(stop - _start);
      std::string time_unit = " ms";
      double elapsed_time = duration.count();
      if (elapsed_time <= std::numeric_limits<double>::epsilon())
      {
        duration = std::chrono::duration<double, std::micro>(stop - _start);
        time_unit = " us";
      }
      std::string time_msg;
      if (elapsed_time > BIG_TIME_VALUE)
        time_msg = "not available";
      time_msg = std::to_string(elapsed_time) + time_unit;
      RCLCPP_INFO_STREAM(rclcpp::get_logger("timer"), "[EXECUTION TIME] " << _name << ": " << time_msg);
    }
    using SharedPtr = std::shared_ptr<Timer>;

  private:
    // CLI_LOCAL
    std::chrono::time_point<std::chrono::system_clock> _tic()
    {
      return std::chrono::system_clock::now();
    }

    std::string _name;
    std::chrono::time_point<std::chrono::system_clock> _start;

    const double BIG_TIME_VALUE = 1000000.0;
  };

} // namespace cli

#endif // CLI__UTIL_HPP_
