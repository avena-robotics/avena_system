#include <chrono>
#include <atomic>
using namespace std::chrono;
enum class TimerStatus : uint8_t
{
    RUNNING = 0,
    STOPPED,
    NOT_STARTED,
    RESETED
};
class Timer
{
private:
    system_clock::time_point _start_time;
    bool _timer_stopped;
    double _elapsed_time;

public:
    TimerStatus _status;
    Timer()
    {
        init();
    }
    void init()
    {
        _timer_stopped = true;
        _elapsed_time = 0.0;
        _status = TimerStatus::NOT_STARTED;
    }
    void start()
    {
        _start_time = system_clock::now();
        _timer_stopped = false;
        _status = TimerStatus::RUNNING;
    }
    void stop()
    {
        _timer_stopped = true;
        _status = TimerStatus::STOPPED;
    }
    void reset()
    {
        _elapsed_time = 0.0;
        _timer_stopped = true;
        _status = TimerStatus::RESETED;
    }
    double read() // in callback, all the time
    {
        if (!_timer_stopped)
        {
            _elapsed_time = duration_cast<milliseconds>(system_clock::now() - _start_time).count()/1000.0;
        }
        return _elapsed_time;
    }
};