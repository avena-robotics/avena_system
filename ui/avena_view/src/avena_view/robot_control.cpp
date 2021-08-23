#include "avena_view/robot_control.h"

RobotControl::RobotControl(Ui::AvenaViewWidget *ui_ptr, std::shared_ptr<rclcpp::Node> node_shared_ptr)
    : _ui_ptr(ui_ptr), _node_shared_ptr(node_shared_ptr), _throttle_value(0)
{
    connectSlotsAndSignals();
    _arm_command_client_ = _node_shared_ptr->create_client<custom_interfaces::srv::ControlCommand>("arm_controller/commands");
    _set_time_factor_pub = _node_shared_ptr->create_publisher<std_msgs::msg::Float64>("/arm_controller/time_factor", 10);

}
RobotControl::~RobotControl()
{
}

void RobotControl::sendArmCommand(ControlCommands command)
{
    int waiting_counter = 0;
    auto request = std::make_shared<custom_interfaces::srv::ControlCommand::Request>();
    request->command = static_cast<int>(command);
    while (waiting_counter++ < 3 && !_arm_command_client_->wait_for_service(std::chrono::milliseconds(100)))
    {
        if (!rclcpp::ok())
        {

            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            writeToConsole("Interrupted while waiting for the service. Exiting.", _ui_ptr->visionAndRobotConsoleLog);
            return;
        }

        // setIndicator("WAITING", Qt::yellow, arm_control_graphics_scene_.get(), ui_.statusLabelArmControl);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        writeToConsole("service not available, waiting again...", _ui_ptr->visionAndRobotConsoleLog);
    }

    if (waiting_counter < 3)
    {
        auto result = _arm_command_client_->async_send_request(request);
        std::future_status status = result.wait_for(std::chrono::milliseconds(100));
        if (status == std::future_status::ready)
        {
            RCLCPP_INFO_STREAM(_node_shared_ptr->get_logger(), "ArmController response: " << result.get()->feedback);
            writeToConsole("ArmController response: " + (result.get()->feedback), _ui_ptr->visionAndRobotConsoleLog);
            // setIndicator("SUCCESS", Qt::green, arm_control_graphics_scene_.get(), ui_.statusLabelArmControl);
        }
        else
        {
            // setIndicator("ERROR", Qt::darkRed, arm_control_graphics_scene_.get(), ui_.statusLabelArmControl);
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service ");
            writeToConsole("Failed to call service", _ui_ptr->visionAndRobotConsoleLog);
        }
    }
    else
    {
        // setIndicator("ERROR", Qt::darkRed, arm_control_graphics_scene_.get(), ui_.statusLabelArmControl);
        writeToConsole("Request timed out", _ui_ptr->visionAndRobotConsoleLog);
    }
}

void RobotControl::runGeneratePath()
{
    writeToConsole("Generating path", _ui_ptr->visionAndRobotConsoleLog);
}
void RobotControl::runGenerateTrajcetory()
{
    writeToConsole("Generating trajectory", _ui_ptr->visionAndRobotConsoleLog);
}
void RobotControl::changeThrottle(int value)
{
    _ui_ptr->throttleLabel->setText("Throttle:   " + QString::number(value) + "%");
    _throttle_value = value;
    std_msgs::msg::Float64 msg;
    msg.data = _throttle_value;
    _set_time_factor_pub->publish(msg);
}
void RobotControl::runExecuteMovement()
{
    writeToConsole("Executing movement", _ui_ptr->visionAndRobotConsoleLog);
    sendArmCommand(ControlCommands::EXECUTE);
}
void RobotControl::runStopMovement()
{
    writeToConsole("Stopping movement", _ui_ptr->visionAndRobotConsoleLog);
    sendArmCommand(ControlCommands::STOP);
}
void RobotControl::runPauseMovement()
{
    writeToConsole("Pausing movement", _ui_ptr->visionAndRobotConsoleLog);
    sendArmCommand(ControlCommands::PAUSE);
}
void RobotControl::runResumeMovement()
{
    writeToConsole("Resuming movement", _ui_ptr->visionAndRobotConsoleLog);
    sendArmCommand(ControlCommands::RESUME);
}

void RobotControl::connectSlotsAndSignals()
{
    connect(_ui_ptr->throttleSlider, SIGNAL(valueChanged(int)), this, SLOT(changeThrottle(int)));
    connect(_ui_ptr->generatePathButton, SIGNAL(clicked(bool)), this, SLOT(runGeneratePath()));
    connect(_ui_ptr->generateTrajectoryButton, SIGNAL(clicked(bool)), this, SLOT(runGenerateTrajcetory()));
    connect(_ui_ptr->executeMovementButton, SIGNAL(clicked(bool)), this, SLOT(runExecuteMovement()));
    connect(_ui_ptr->stopMovementButton, SIGNAL(clicked(bool)), this, SLOT(runStopMovement()));
    connect(_ui_ptr->pauseMovementButton, SIGNAL(clicked(bool)), this, SLOT(runPauseMovement()));
    connect(_ui_ptr->resumeMovementButton, SIGNAL(clicked(bool)), this, SLOT(runResumeMovement()));
}