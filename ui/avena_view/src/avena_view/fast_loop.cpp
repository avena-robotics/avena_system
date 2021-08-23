#include "avena_view/fast_loop.h"

FastLoop::FastLoop(Ui::AvenaViewWidget *ui_ptr, std::shared_ptr<rclcpp::Node> node_shared_ptr, QWidget* widget)
    : _ui_ptr(ui_ptr), _node_shared_ptr(node_shared_ptr), _widget(widget)
{
    qRegisterMetaType<std_msgs::msg::Bool::SharedPtr>("std_msgs::msg::Bool::SharedPtr");

    setUpLeds();
    setUpRosComponents();
    connectSlotsToSignals();
    setUpDangerToolTimer();
}

FastLoop::~FastLoop()
{
    _ui_ptr = nullptr;
    _node_shared_ptr = nullptr;
    _widget = nullptr;
}

void FastLoop::setUpRosComponents()
{
    _danger_tool_in_hand_pub = _node_shared_ptr->create_publisher<std_msgs::msg::Bool>("/danger_tool_in_hand", 10);
    _set_security_background_pub = _node_shared_ptr->create_publisher<std_msgs::msg::Bool>("/rgbdiff_set_background", 10);
    _set_tracking_background_pub = _node_shared_ptr->create_publisher<std_msgs::msg::Bool>("/tracking_set_background", 10);
    _tracking_trigger_sub = _node_shared_ptr->create_subscription<std_msgs::msg::Bool>(
        "/operational_area_trigger",
        rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&FastLoop::trackingTriggerStatusCallback, this, std::placeholders::_1));
    _danger_tool_in_hand_sub = _node_shared_ptr->create_subscription<std_msgs::msg::Bool>(
        "/danger_tool_in_hand",
        rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&FastLoop::dangerToolStatusCallback, this, std::placeholders::_1));
    _security_trigger_sub = _node_shared_ptr->create_subscription<std_msgs::msg::Bool>(
        "/security_trigger",
        rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&FastLoop::securityTriggerStatusCallback, this, std::placeholders::_1));
    _security_pause_sub = _node_shared_ptr->create_subscription<std_msgs::msg::Bool>(
        "/security_pause",
        rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&FastLoop::securityPauseStatusCallback, this, std::placeholders::_1));
}

void FastLoop::setSecurityBackground()
{
    std_msgs::msg::Bool msg;
    msg.data = true;
    _set_security_background_pub->publish(msg);
    writeToConsole("Set security background published", _ui_ptr->fastLoopLogConsole);
}
void FastLoop::setTrackingBackground()
{
    std_msgs::msg::Bool msg;
    msg.data = true;
    _set_security_background_pub->publish(msg);
    writeToConsole("Set tracking background published", _ui_ptr->fastLoopLogConsole);
}
void FastLoop::switchDangerTool()
{
    _danger_tool_status = !_danger_tool_status;
    writeToConsole("Danger tool in hand status changed: " + std::to_string(_danger_tool_status), _ui_ptr->fastLoopLogConsole);
}

void FastLoop::setSecurityTriggerLed(std_msgs::msg::Bool::SharedPtr msg)
{
    if(msg->data)
    {
        setLed(QColor("green"), _security_trigger_led_scene);
    }
    else
    {
        setLed(QColor("red"), _security_trigger_led_scene);
    }
}
void FastLoop::setSecurityPauseLed(std_msgs::msg::Bool::SharedPtr msg)
{
    if(msg->data)
    {
        setLed(QColor("green"), _security_pause_led_scene);
    }
    else
    {
        setLed(QColor("red"), _security_pause_led_scene);
    }
}
void FastLoop::setDangerToolLed(std_msgs::msg::Bool::SharedPtr msg)
{
    if(msg->data)
    {
        setLed(QColor("green"), _danger_tool_led_scene);
    }
    else
    {
        setLed(QColor("red"), _danger_tool_led_scene);
    }
}
void FastLoop::setOperationalAreaTriggerLed(std_msgs::msg::Bool::SharedPtr msg)
{
    if(msg->data)
    {
        setLed(QColor("green"), _tracking_trigger_led_scene);
    }
    else
    {
        setLed(QColor("red"), _tracking_trigger_led_scene);
    }
}

void FastLoop::setUpDangerToolTimer()
{
    _danger_tool_status = false;
    _danger_tool_timer = std::make_shared<QTimer>();
    connect(_danger_tool_timer.get(), SIGNAL(timeout()), this, SLOT(publishDangerToolStatus()));
    _danger_tool_timer->start(DANGER_TOOL_PUBLISHING_FREQ);
}

void FastLoop::publishDangerToolStatus()
{
    std_msgs::msg::Bool msg;
    msg.data = _danger_tool_status;
    _danger_tool_in_hand_pub->publish(msg);
}

void FastLoop::connectSlotsToSignals()
{
    connect(_ui_ptr->dangerToolSwitch, SIGNAL(clicked(bool)), this, SLOT(switchDangerTool()));
    connect(_ui_ptr->setSecurityBackgoundButton, SIGNAL(clicked(bool)), this, SLOT(setSecurityBackground()));
    connect(_ui_ptr->setTrackingBackgroundButton, SIGNAL(clicked(bool)), this, SLOT(setTrackingBackground()));

    connect(this, SIGNAL(dangerToolChanged(std_msgs::msg::Bool::SharedPtr)), this, SLOT(setDangerToolLed(std_msgs::msg::Bool::SharedPtr)));
    connect(this, SIGNAL(securityTriggerChanged(std_msgs::msg::Bool::SharedPtr)), this, SLOT(setSecurityTriggerLed(std_msgs::msg::Bool::SharedPtr)));
    connect(this, SIGNAL(securityPauseChanged(std_msgs::msg::Bool::SharedPtr)), this, SLOT(setSecurityPauseLed(std_msgs::msg::Bool::SharedPtr)));
    connect(this, SIGNAL(trackingTriggerChanged(std_msgs::msg::Bool::SharedPtr)), this, SLOT(setOperationalAreaTriggerLed(std_msgs::msg::Bool::SharedPtr)));
}

void FastLoop::dangerToolStatusCallback(std_msgs::msg::Bool::SharedPtr msg)
{
    emit dangerToolChanged(msg);
}

void FastLoop::securityTriggerStatusCallback(std_msgs::msg::Bool::SharedPtr msg)
{
    emit securityTriggerChanged(msg);
}

void FastLoop::securityPauseStatusCallback(std_msgs::msg::Bool::SharedPtr msg)
{
    emit securityPauseChanged(msg);
}

void FastLoop::trackingTriggerStatusCallback(std_msgs::msg::Bool::SharedPtr msg)
{
    emit trackingTriggerChanged(msg);
}

void FastLoop::setUpLeds()
{
    _danger_tool_led_scene = std::make_shared<QGraphicsScene>(_widget);
    _security_pause_led_scene = std::make_shared<QGraphicsScene>(_widget);
    _security_trigger_led_scene = std::make_shared<QGraphicsScene>(_widget);
    _tracking_trigger_led_scene = std::make_shared<QGraphicsScene>(_widget);

    _ui_ptr->dangerToolLed->setScene(_danger_tool_led_scene.get());
    _ui_ptr->securityPauseLed->setScene(_security_pause_led_scene.get());
    _ui_ptr->securityTriggerLed->setScene(_security_trigger_led_scene.get());
    _ui_ptr->operationalAreaTriggerLed->setScene(_tracking_trigger_led_scene.get());
}
void FastLoop::setLed(const QColor& color, std::shared_ptr<QGraphicsScene> led)
{
    QBrush led_brush(color);
    led->addEllipse(0,0,45,45, QPen(), led_brush);
}

