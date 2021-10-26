#include "helpers_commons/watchdog.hpp"

namespace helpers
{
    Watchdog::Watchdog(rclcpp::Node *node, helpers::WatchdogInterface *gui_interface, const std::string &system_monitor_name)
        : _node(node),
          _gui_interface(gui_interface),
          _pub_heartbeat_duration(1.0 / 10.0),
          _pub_statistics_duration(1.0 / 10.0),
          _system_check_duration(1.0 / 5.0),
          _system_monitor_name(system_monitor_name),
          _working(true)
    {
        _action_started = false;
        _action_time_period = 0;
        _node_name = _node->get_name();
        RCLCPP_INFO_STREAM(rclcpp::get_logger(_node_name), "Initialization watchdog for module \"" << _node_name << "\"");

        auto qos_settings = rclcpp::QoS(rclcpp::KeepLast(1));
        _sub_watchdog = _node->create_subscription<custom_interfaces::msg::Watchdog>("system_monitor/watchdog", qos_settings, std::bind(&Watchdog::_watchdogCallback, this, std::placeholders::_1));
        _sub_command = _node->create_subscription<custom_interfaces::msg::ModuleCommand>("system_monitor/command", qos_settings, std::bind(&Watchdog::_commandCallback, this, std::placeholders::_1));
        
        _pub_heartbeat = _node->create_publisher<custom_interfaces::msg::Heartbeat>("system_monitor/heartbeat", qos_settings);
        _pub_statistics = _node->create_publisher<custom_interfaces::msg::Statistics>("system_monitor/statistics", qos_settings);

        // Timer to publish heartbeat periodically
        _heart_beat_thread = std::thread(std::bind(&Watchdog::_spinHeartBeat,this));
        
        // Timer to publish statistics periodically
        _statistics_thread = std::thread(std::bind(&Watchdog::_spinStatistics, this));
        // _pub_statistics_timer = _node->create_wall_timer(_pub_statistics_duration, std::bind(&Watchdog::_broadcastStatistics, this));

        // System check
        _system_check_timer = _node->create_wall_timer(_system_check_duration, std::bind(&Watchdog::_systemCheckTimerCallback, this));
    }

    Watchdog::~Watchdog()
    {
        if(_heart_beat_thread.joinable())
        {
            _heart_beat_thread.join();
        }

        if(_statistics_thread.joinable())
        {
            _statistics_thread.join();
        }
    }

    void Watchdog::_spinHeartBeat()
    {
        rclcpp::Rate rate(100);
        while(rclcpp::ok())
        {
            _broadcastHeartbeat();
            rate.sleep();
        }
    }

        void Watchdog::_spinStatistics()
    {
        
        rclcpp::Rate rate(1000/std::chrono::duration_cast<std::chrono::milliseconds>(_pub_statistics_duration).count());
        while(rclcpp::ok())
        {
            _broadcastStatistics();
            rate.sleep();
        }
    }

    void Watchdog::_broadcastHeartbeat()
    {
        // ProcStat * ps = get_proc_stat_info();
        custom_interfaces::msg::Heartbeat::SharedPtr msg(new custom_interfaces::msg::Heartbeat);
        msg->header.frame_id = "world";
        msg->header.stamp = _node->now();
        msg->module_name = _node_name;
        msg->status = _gui_interface->status;
        this->_pub_heartbeat->publish(*msg);
    }

    void Watchdog::_commandCallback(const custom_interfaces::msg::ModuleCommand::SharedPtr command)
    {
        if (_node_name != command->module_name)
        {
            return;
        }

        switch (command->command)
        {
        case custom_interfaces::msg::ModuleCommand::START:
            if (_gui_interface->status == custom_interfaces::msg::Heartbeat::STOPPED)
            {
                _gui_interface->initNode();
            }
            else
            {
                RCLCPP_WARN(_node->get_logger(), "Module is not stopped, so it can't be started");
            }
            break;
        case custom_interfaces::msg::ModuleCommand::STOP:
            RCLCPP_INFO(_node->get_logger(), "Stopping module");
            if (_gui_interface->status == custom_interfaces::msg::Heartbeat::RUNNING)
            {
                _gui_interface->shutDownNode();
            }
            else
            {
                RCLCPP_WARN(_node->get_logger(), "Module is not started, so it can't be stopped");
            }
            break;
        default:
            RCLCPP_ERROR(_node->get_logger(), "Unknown command");
            break;
        }
    }

    void Watchdog::_broadcastStatistics()
    {
        ProcStat *ps = get_proc_stat_info();
        Uptime *up = get_uptime_info();
        custom_interfaces::msg::Statistics::SharedPtr msg(new custom_interfaces::msg::Statistics);
        double current_cpu_jiffies = static_cast<double>(get_total_cpu_jiffies());
        double current_proc_jiffies = static_cast<double>(get_proc_jiffies(false));
        double cpu_count = static_cast<double>(get_cpu_count());
        double diff_denominator = (current_cpu_jiffies - this->_prev_cpu_jiffies) / cpu_count;
        double proc_util = 100 * (current_proc_jiffies - this->_prev_proc_jiffies) / diff_denominator;
        double memory_consumption_mb = static_cast<double>(get_process_memory_load_kb()) / 1024.0;
        // std::cout<<current_proc_jiffies - this->_prev_proc_jiffies<<" "<<diff<<std::endl;
        msg->header.frame_id = "world";
        msg->header.stamp = _node->now();
        msg->module_name = _node_name;

        msg->utime = ps->utime;
        msg->stime = ps->stime;
        msg->cutime = ps->cutime;
        msg->cstime = ps->cstime;
        msg->rss = ps->rss;
        msg->vsize = ps->vsize;
        msg->cpu_jiffies = current_cpu_jiffies;
        msg->cpu_util = proc_util;
        msg->memory_load = memory_consumption_mb;
        delete_proc_stat_info(ps);
        delete_uptime_info(up);
        this->_prev_proc_jiffies = current_proc_jiffies;
        this->_prev_cpu_jiffies = current_cpu_jiffies;
        auto timestamp = std::chrono::system_clock::now();
        if (this->isActionStarted())
            this->_action_time_period += std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - this->_start_action_timestamp).count();
        msg->action_time = this->_action_time_period;

        this->_start_action_timestamp = timestamp;
        this->_action_time_period = 0;
        // std::string node_name = "change_detect";
        // std::string node_name_1 = "robot_self_filter";
        // std::string node_name_2 = "ptcld_transformer";

        // std::cout<<this->_node->get_name()<<std::endl;
        // if(std::string(this->_node->get_name()) == node_name_1 /*|| std::string(this->_node->get_name()) == node_name_1 || std::string(this->_node->get_name()) == node_name_2*/){
        this->_pub_statistics->publish(*msg);
        // }
    }

    void Watchdog::_watchdogCallback(const custom_interfaces::msg::Watchdog::SharedPtr /*watchdog_data*/)
    {
        this->_system_check_timer->reset();
        this->_working = true;
    }

    bool Watchdog::isWorking()
    {
        return this->_working;
    }

    bool Watchdog::isActionStarted()
    {
        return this->_action_started;
    }

    void Watchdog::beginAction()
    {
        this->_start_action_timestamp = std::chrono::system_clock::now();
        this->_action_started = true;
    }

    void Watchdog::endAction()
    {
        auto timestamp = std::chrono::system_clock::now();
        this->_action_time_period += std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - this->_start_action_timestamp).count();
        this->_action_started = false;
    }

    void Watchdog::_systemCheckTimerCallback()
    {
        // RCLCPP_ERROR(_node_handler->get_logger(), "WATCHDOG");
        this->_working = false;
    }

} // namespace helpers
