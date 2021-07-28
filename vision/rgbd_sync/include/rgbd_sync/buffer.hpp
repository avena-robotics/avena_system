#ifndef SCENE_PUBLISHER__BUFFER_HPP_
#define SCENE_PUBLISHER__BUFFER_HPP_

// ___CPP___
#include <chrono>
#include <vector>
#include <mutex>

// ___ROS___
#include <rclcpp/rclcpp.hpp>

namespace scene_publisher
{
  template <typename MsgType>
  class Buffer
  {
  public:
    Buffer() = default;

    explicit Buffer(const std::chrono::duration<double> &cutoff_time)
        : _cutoff_time(cutoff_time)
    {
    }
    ~Buffer() = default;
    Buffer &operator=(const Buffer &other)
    {
      // No need to lock because this operator will be used only in constructor of object
      // which contains this buffer, during runtime it is not desired to reassign buffer
      if (this != &other)
      {
        _cutoff_time = other._cutoff_time;
        _buffered_data = other._buffered_data;
      }
      return *this;
    }

    void push(const MsgType &data)
    {
      std::lock_guard<std::mutex> lg(_data_mutex);
      if (_buffered_data.size() == _buffered_data.capacity())
        _buffered_data.reserve(_buffered_data.size() + 10);
      _buffered_data.push_back(data);
      _removeOldData();
    }

    MsgType &getData(const std::chrono::duration<double> &timestamp)
    {
      std::lock_guard<std::mutex> lg(_data_mutex);
      auto found_data = std::find_if(_buffered_data.rbegin(), _buffered_data.rend(),
                                     [this, &timestamp](const MsgType &data)
                                     {
                                       auto data_stamp = std::chrono::duration<double>(_rosTimeToSeconds(data.header.stamp));
                                       return data_stamp == timestamp;
                                     });
      if (found_data == _buffered_data.rend())
        throw std::range_error("There is no buffered message with timestamp " + std::to_string(timestamp.count()) + " sec");
      return *found_data;
    }

  private:
    std::chrono::duration<double> _cutoff_time;
    std::vector<MsgType> _buffered_data;
    std::mutex _data_mutex;

    void _removeOldData()
    {
      // If cutoff time is non positive, store all messages
      if (_cutoff_time.count() <= 0.0)
        return;
      auto now_sec = _nowInSeconds();
      for (auto it = _buffered_data.begin(); it != _buffered_data.end(); ++it)
      {
        std::chrono::duration<double> data_time_sec(_rosTimeToSeconds(it->header.stamp));
        if (now_sec - data_time_sec > _cutoff_time)
          it = _buffered_data.erase(it);
      }
    }

    double _rosTimeToSeconds(const builtin_interfaces::msg::Time &time)
    {
      return time.sec + time.nanosec / 1e9;
    }

    std::chrono::duration<double> _nowInSeconds()
    {
      auto now = std::chrono::system_clock::now().time_since_epoch();
      auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(now);
      return now_sec;
    }
  };
} // namespace scene_publisher

#endif // SCENE_PUBLISHER__BUFFER_HPP_
