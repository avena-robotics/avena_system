#include "logic_bt/logic_bt.hpp"
namespace logic_bt
{
    Logic::Logic(const rclcpp::NodeOptions &options) : Node("logic", options)
    {
        status = custom_interfaces::msg::Heartbeat::STOPPED;
        RCLCPP_INFO(this->get_logger(), "started Node");
        _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
        using namespace std::placeholders;
        RCLCPP_INFO(this->get_logger(), "server created");
        this->logic_server_ = rclcpp_action::create_server<PickAndPlaceAction>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "logic_server",
            std::bind(&Logic::_handle_goal, this, _1, _2),
            std::bind(&Logic::_handle_cancel, this, _1),
            std::bind(&Logic::_handle_accepted, this, _1));
        _timer_pub = this->create_publisher<std_msgs::msg::Float64>("timer_reading", 10);
        _timer_sub = this->create_subscription<std_msgs::msg::String>(
            "timer_command", 10, std::bind(&Logic::_timer_callback, this, _1));
        // BT::SharedLibrary loader;
        signal(SIGINT, _signalHandler);
    }
    void Logic::_signalHandler(int signum)
    {
        (void)signum;
        rclcpp::shutdown();
        // supposed to be signum
        exit(0);
    }

    void Logic::initNode()
    {
        status = custom_interfaces::msg::Heartbeat::STARTING;
        BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
        blackboard->set<rclcpp::Node::SharedPtr>("node", shared_from_this());
        BT::SharedLibrary loader;
        std::filesystem::path plugin_share_directory(ament_index_cpp::get_package_share_directory("logic_bt_nodes"));
        std::vector<std::string> plugin_paths;
        for (const auto &entry : std::filesystem::directory_iterator(plugin_share_directory.parent_path().parent_path().string() + "/lib"))
        {
            plugin_paths.emplace_back(entry.path());
        }
        for (const auto &plugin_path : plugin_paths)
        {
            _factory.registerFromPlugin(plugin_path);
        }
        // _factory.registerFromPlugin("/root/ros2_ws/install/logic_bt/lib/libplugin.so");
        std::string tree_xml_path = ament_index_cpp::get_package_share_directory("logic_bt") + "/xml/main_tree.xml";
        _logic_tree = _factory.createTreeFromFile(tree_xml_path, blackboard);
        _publisher_zmq_ptr = std::make_shared<BT::PublisherZMQ>(_logic_tree);

        status = custom_interfaces::msg::Heartbeat::RUNNING;
    }

    void Logic::shutDownNode()
    {
        RCLCPP_INFO(this->get_logger(), "shut Down Node");
        if (status != custom_interfaces::msg::Heartbeat::STOPPED)
            status = custom_interfaces::msg::Heartbeat::STOPPED;
    }
    Logic::~Logic()
    {
        shutDownNode();
    }
    rclcpp_action::GoalResponse Logic::_handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const PickAndPlaceAction::Goal> goal)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Received request to " << goal->command);
        (void)uuid;

        if (std::find(_commands.begin(), _commands.end(), goal->command) != _commands.end())
        {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        else // Let's reject unsupported commands
        {
            RCLCPP_ERROR(this->get_logger(), "unsupported command, only use {start, pause, resume, stop}");
            return rclcpp_action::GoalResponse::REJECT;
        }
    }
    rclcpp_action::CancelResponse Logic::_handle_cancel(
        const std::shared_ptr<GoalHandlePickAndPlaceAction> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void Logic::_handle_accepted(const std::shared_ptr<GoalHandlePickAndPlaceAction> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&Logic::_execute, this, _1), goal_handle}.detach();
    }

    void Logic::_execute(const std::shared_ptr<GoalHandlePickAndPlaceAction> goal_handle)
    {
        // init the timer for each execute
        _timer.init();
        auto result = std::make_shared<PickAndPlaceAction::Result>();
        if (status != custom_interfaces::msg::Heartbeat::RUNNING)
        {
            result->result = false;
            RCLCPP_INFO(this->get_logger(), "Stopped [server is not available]");
            goal_handle->abort(result);
            return;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Executing goal");
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<PickAndPlaceAction::Feedback>();
            RCLCPP_INFO_STREAM(this->get_logger(), "what to pick: " << goal->what_to_pick << ", where to put: " << goal->where_to_put);
            for (const auto &node : _logic_tree.nodes)
            {
                if (node->name() == "GUIParamsWriter")
                {
                    node->setOutput<std::string>("what_to_pick", goal->what_to_pick);
                    node->setOutput<std::string>("where_to_put", goal->where_to_put);
                }
            }
            BT::NodeStatus logic_tree_result;
            double tick_elapsed_time{0.0}, desired_loop_time{1.0}; // ms
            int actual_loop_time{1000};                              // us
            do
            {
                auto timer_message = std_msgs::msg::Float64();
                timer_message.data = _timer.read();
                // RCLCPP_INFO_STREAM(this->get_logger(), "Timer Reading: " << timer_message.data);
                _timer_pub->publish(timer_message);

                // Check if there is a cancel request
                if (goal_handle->is_canceling())
                {
                    result->result = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal Canceled");
                    return;
                }
                else
                {
                    auto start = std::chrono::system_clock::now();
                    logic_tree_result = _logic_tree.tickRoot();
                    tick_elapsed_time = std::chrono::duration<double, std::micro>(std::chrono::system_clock::now() - start).count();
                    // RCLCPP_INFO_STREAM(this->get_logger(), "tick elapsed time (  " << tick_elapsed_time << " ms )");
                }

                // Update and Publish feedback
                feedback->status = _bt_states.at(logic_tree_result);
                // RCLCPP_INFO(this->get_logger(), "Publish Feedback");
                // if (logic_tree_result == BT::NodeStatus::SUCCESS)
                // {
                //     result->result = true;
                //     goal_handle->succeed(result);
                //     RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
                //     return;
                // }
                // else if (logic_tree_result == BT::NodeStatus::FAILURE)
                // {
                //     result->result = false;
                //     RCLCPP_INFO(this->get_logger(), "Goal Failed");
                //     goal_handle->abort(result);
                //     return;
                // }
                // rclcpp::spin_some(this->get_node_base_interface());
                if (tick_elapsed_time < desired_loop_time * 1000.0)
                {
                    actual_loop_time = static_cast<int>(desired_loop_time * 1000.0 - tick_elapsed_time);
                    // RCLCPP_INFO_STREAM(this->get_logger(), "tick elapsed time (  " << actual_loop_time/1000.0 << " ms )");
                    std::this_thread::sleep_for(std::chrono::microseconds(actual_loop_time));
                }
                // else
                // {
                //     result->result = false;
                //     RCLCPP_INFO_STREAM(this->get_logger(), "Very long ticking time ( > " << desired_loop_time << " ms ), Aborted");
                //     goal_handle->abort(result);
                //     return;
                // }

                // } while (logic_tree_result == BT::NodeStatus::RUNNING);
                // if (logic_tree_result == BT::NodeStatus::SUCCESS)
                //     _timer.init();

            } while (true && rclcpp::ok());

            // for zmq
            std::this_thread::sleep_for(Milliseconds(500));
        }
    }
    void Logic::_timer_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg)
        {
            if (msg->data == "reset")
            {
                // RCLCPP_INFO_STREAM(this->get_logger(), "Timer is Set to zero");
                if (_timer._status != TimerStatus::RESETED)
                {
                    _timer.reset();
                }
            }
            else if (msg->data == "stop")
            {
                // RCLCPP_INFO_STREAM(this->get_logger(), "Timer is stopped");
                if (_timer._status != TimerStatus::STOPPED)
                {
                    _timer.stop();
                }
            }
            else if (msg->data == "start")
            {
                // RCLCPP_INFO_STREAM(this->get_logger(), "Timer is started");
                if (_timer._status != TimerStatus::RUNNING)
                {
                    _timer.start();
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unsupported Timer Command, only [set,stop,start]");
            }
        }
    }

} // end of logic_bt
// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<logic_bt::Logic>());
//     // logic_node->run();
//     // rclcpp::shutdown();
//     return 0;
// }
// //
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(logic_bt::Logic)
