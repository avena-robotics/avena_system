#pragma once

#include <QTimer>
#include <ui_avena_view.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <avena_view/config.h>
#include <avena_view/utils.h>
#include "std_msgs/msg/bool.hpp"

Q_DECLARE_METATYPE(std_msgs::msg::Bool::SharedPtr)

class FastLoop : public QObject
{
    Q_OBJECT
public:
    FastLoop(Ui::AvenaViewWidget *ui_ptr, std::shared_ptr<rclcpp::Node> node_shared_ptr, QWidget* widget);
    ~FastLoop();

private slots:
    void setSecurityBackground();
    void setTrackingBackground();
    void switchDangerTool();
    void setSecurityTriggerLed(std_msgs::msg::Bool::SharedPtr msg);
    void setSecurityPauseLed(std_msgs::msg::Bool::SharedPtr msg);
    void setDangerToolLed(std_msgs::msg::Bool::SharedPtr msg);
    void setOperationalAreaTriggerLed(std_msgs::msg::Bool::SharedPtr msg);
    void publishDangerToolStatus();

signals:
    void securityTriggerChanged(std_msgs::msg::Bool::SharedPtr msg);
    void securityPauseChanged(std_msgs::msg::Bool::SharedPtr msg);
    void dangerToolChanged(std_msgs::msg::Bool::SharedPtr msg);
    void trackingTriggerChanged(std_msgs::msg::Bool::SharedPtr msg);

private:
    void setUpDangerToolTimer();
    void connectSlotsToSignals();
    void setUpRosComponents();

    void dangerToolStatusCallback(std_msgs::msg::Bool::SharedPtr msg);
    void securityTriggerStatusCallback(std_msgs::msg::Bool::SharedPtr msg);
    void securityPauseStatusCallback(std_msgs::msg::Bool::SharedPtr msg);
    void trackingTriggerStatusCallback(std_msgs::msg::Bool::SharedPtr msg);

    void setUpLeds();
    void setLed(const QColor& color, std::shared_ptr<QGraphicsScene> led);

    Ui::AvenaViewWidget *_ui_ptr;
    std::shared_ptr<rclcpp::Node> _node_shared_ptr;
    QWidget* _widget;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _danger_tool_in_hand_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _set_security_background_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _set_tracking_background_pub;


    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _security_trigger_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _security_pause_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _danger_tool_in_hand_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _tracking_trigger_sub;


    std::shared_ptr<QGraphicsScene> _danger_tool_led_scene;
    std::shared_ptr<QGraphicsScene> _security_trigger_led_scene;
    std::shared_ptr<QGraphicsScene> _security_pause_led_scene;
    std::shared_ptr<QGraphicsScene> _tracking_trigger_led_scene;

    bool _danger_tool_status;
    std::shared_ptr<QTimer> _danger_tool_timer;
};