#ifndef HELPERS_COMMONS__TIMERS_HPP_
#define HELPERS_COMMONS__TIMERS_HPP_

// ___CPP___
#include <memory>
#include <chrono>
#include <iostream>
#include <cstring>

// ___ROS___
#include <rclcpp/logging.hpp>

namespace helpers
{
  class TicTok
  {
    using clk = std::chrono::high_resolution_clock;

  private:
    std::chrono::_V2::system_clock::time_point t1;
    // std::chrono::_V2::system_clock::time_point t2;
    bool cant_touch_this = false;

    TicTok() {}
    inline static TicTok *_instance;

  public:
    TicTok(TicTok &other) = delete;
    void operator=(const TicTok &) = delete;

    void start()
    {
      if (!cant_touch_this)
      {
        t1 = std::chrono::high_resolution_clock::now();
        cant_touch_this = true;
      }
    }

    void stop()
    {
      auto t2 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> fp_ms = t2 - t1;

      // integral duration: requires duration_cast
      auto int_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);

      // converting integral duration to integral duration of shorter divisible time unit:
      // no duration_cast needed
      std::chrono::duration<long, std::micro> int_usec = int_ms;

      // std::cout << "Pipeline took " << fp_ms.count() << " [ms]" << std::endl;
      cant_touch_this = false;
    }

    static TicTok *GetInstance()
    {
      if (_instance == nullptr)
      {
        _instance = new TicTok();
      }
      return _instance;
    }
  };

  class Timer
  {
  public:
    using clk = std::chrono::steady_clock;

    explicit Timer(const std::string &message, bool debug = false)
        : _start(clk::now()), _message(message), _debug(debug), _logger(rclcpp::get_logger("null")) {}

    explicit Timer(const std::string &message, rclcpp::Logger logger)
        : _start(clk::now()), _message(message), _logger(logger) {}

    Timer(const Timer &other) = delete;

    ~Timer()
    {
      auto stop = clk::now();
      auto duration = std::chrono::duration<double, std::milli>(stop - _start);
      std::string time_unit = " ms";
      if (duration.count() <= std::numeric_limits<double>::epsilon())
      {
        duration = std::chrono::duration<double, std::micro>(stop - _start);
        time_unit = " us";
      }
      if (std::strcmp(_logger.get_name(), "null") == 0)
      {
        if (_debug)
          std::cout << "[EXECUTION TIME]: " << _message << ": " << duration.count() << time_unit << std::endl;
      }
      else
        RCLCPP_DEBUG_STREAM(_logger, "[EXECUTION TIME]: " << _message << ": " << duration.count() << time_unit);
    }

  private:
    std::chrono::time_point<clk> _start;
    std::string _message;
    bool _debug;
    rclcpp::Logger _logger;
  };

  class TimerROS
  {
  public:
    explicit TimerROS(const std::string &message, rclcpp::Logger logger)
        : _start(std::chrono::system_clock::now()), _message(message), _logger(logger) {}

    explicit TimerROS(const std::string &message, const std::string &logger_name)
        : TimerROS(message, rclcpp::get_logger(logger_name)) {}

    TimerROS(const TimerROS &other) = delete;

    ~TimerROS()
    {
      auto stop = std::chrono::system_clock::now();
      auto duration = std::chrono::duration<double, std::milli>(stop - _start);
      std::string time_unit = " ms";
      if (duration.count() <= std::numeric_limits<double>::epsilon())
      {
        duration = std::chrono::duration<double, std::micro>(stop - _start);
        time_unit = " us";
      }
      RCLCPP_DEBUG_STREAM(_logger, "[EXECUTION TIME]: " << _message << ": " << duration.count() << time_unit);
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> _start;
    std::string _message;
    rclcpp::Logger _logger;
  };

} // namespace helpers

#endif // HELPERS_COMMONS__UTILS_HPP_
