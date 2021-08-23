#pragma once

#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <ui_avena_view.h>
#include "custom_interfaces/srv/control_command.hpp"
#include "avena_view/utils.h"
#include "avena_view/config.h"


class RobotControl: public QObject
{
    Q_OBJECT

public:
    RobotControl(Ui::AvenaViewWidget* ui_ptr, std::shared_ptr<rclcpp::Node> node_shared_ptr);
    ~RobotControl();

private slots:
    void runGeneratePath();
    void runGenerateTrajcetory();
    void changeThrottle(int value);
    void runExecuteMovement();
    void runStopMovement();
    void runPauseMovement();
    void runResumeMovement();
    void sendArmCommand(ControlCommands command);
private:
    void connectSlotsAndSignals();

    Ui::AvenaViewWidget* _ui_ptr;
    std::shared_ptr<rclcpp::Node> _node_shared_ptr;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _set_time_factor_pub;

    rclcpp::Client<custom_interfaces::srv::ControlCommand>::SharedPtr _arm_command_client_;

    int _throttle_value;
    
};