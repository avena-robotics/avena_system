#include "arm_controller/trajectory_feeder.hpp"

TrajectoryFeeder::TrajectoryFeeder() : Node("trajectory_feeder")
{

    _controller_state_sub = create_subscription<std_msgs::msg::Int32>("/arm_controller/state", 10, std::bind(&TrajectoryFeeder::_controllerStateCb, this, std::placeholders::_1));
    _start_feeding_sub = create_subscription<std_msgs::msg::Int32>("/start_feeding", 10, std::bind(&TrajectoryFeeder::_startFeedingCb, this, std::placeholders::_1));
    _trajectory_sub = create_subscription<trajectory_msgs::msg::JointTrajectory>("/trajectory", 1000, std::bind(&TrajectoryFeeder::_saveTrajectoryCb, this, std::placeholders::_1));
    _arm_command_client_ = create_client<custom_interfaces::srv::ControlCommand>("arm_controller/commands");
    _trajectory_pub = create_publisher<trajectory_msgs::msg::JointTrajectory>("/trajectory", 1000);

    _loop_timer = this->create_wall_timer(std::chrono::seconds(2), std::bind(&TrajectoryFeeder::feedTrajectories, this));
}

void TrajectoryFeeder::_saveTrajectoryCb(trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    if (!_feed)
    {
        RCLCPP_INFO(this->get_logger(), "Received trajectory, saving.");
        _trajectory_array.push_back(*msg);
    }
}

void TrajectoryFeeder::_controllerStateCb(std_msgs::msg::Int32::SharedPtr msg)
{
    _controller_state = msg->data;
}

void TrajectoryFeeder::_startFeedingCb(std_msgs::msg::Int32::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Starting feeding");
    _feed = msg->data;
}

void TrajectoryFeeder::feedTrajectories()
{
    if (!rclcpp::ok())
        exit(1);
    if (!_feed)
    {
        return;
    }
    if (_controller_state == 4)
    {
        return;
    }
    else if (_controller_state == 3)
    {
        
        RCLCPP_INFO(this->get_logger(), "Sending trajectory %i of %i.", _traj_it, _trajectory_array.size()-1);
        _trajectory_pub->publish(_trajectory_array[_traj_it]);
        _traj_it = (_traj_it + 1) % (_trajectory_array.size());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(this->get_logger(), "Sending execute command");
        sendArmCommand(4);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return;
    }
}

void TrajectoryFeeder::sendArmCommand(int command)
{
    int waiting_counter = 0;
    auto request = std::make_shared<custom_interfaces::srv::ControlCommand::Request>();
    request->command = static_cast<int>(command);
    while (waiting_counter++ < 3 && !_arm_command_client_->wait_for_service(std::chrono::milliseconds(100)))
    {
        if (!rclcpp::ok())
        {

            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }

        // setIndicator("WAITING", Qt::yellow, arm_control_graphics_scene_.get(), ui_.statusLabelArmControl);
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    if (waiting_counter < 3)
    {
        auto result = _arm_command_client_->async_send_request(request);
        std::future_status status = result.wait_for(std::chrono::milliseconds(100));
        if (status == std::future_status::ready)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "ArmController response: " << result.get()->feedback);
            // setIndicator("SUCCESS", Qt::green, arm_control_graphics_scene_.get(), ui_.statusLabelArmControl);
        }
        else
        {
            // setIndicator("ERROR", Qt::darkRed, arm_control_graphics_scene_.get(), ui_.statusLabelArmControl);
            RCLCPP_ERROR(this->get_logger(), "Failed to call service ");
        }
    }
    else
    {
        // setIndicator("ERROR", Qt::darkRed, arm_control_graphics_scene_.get(), ui_.statusLabelArmControl);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<TrajectoryFeeder> feeder;
    feeder = std::make_shared<TrajectoryFeeder>();
    rclcpp::spin(feeder);
    // rclcpp::spin(std::make_shared<ArmInterface>("0AA"));
    return 0;
}