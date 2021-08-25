#pragma once

#include <QObject>

#include <ui_avena_view.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "custom_interfaces/action/simple_action.hpp"

#include "avena_view/utils.h"

Q_DECLARE_METATYPE(std::string)

using namespace std::chrono_literals;
using namespace std::placeholders;

class VisionActions : public QObject
{
    Q_OBJECT
public:
    using SimpleAction = custom_interfaces::action::SimpleAction;
    using GoalSimpleAction = rclcpp_action::ClientGoalHandle<SimpleAction>;

    VisionActions(Ui::AvenaViewWidget *ui_ptr, rclcpp::Node::SharedPtr node_shared_ptr);
    ~VisionActions();

private slots:
    void executeRgbdSync();
    void executeDetect();
    void executeComposeItems();
    void executeEstimateShape();
    void executeGenerateOctomap();
    void writeConsoleMsg(const std::string& msg);

signals:
    void consoleMsgRequested(const std::string& msg);

private:
    void connectSlotsToSignals();

    Ui::AvenaViewWidget *_ui_ptr;
    rclcpp::Node::SharedPtr _node_shared_ptr;

    rclcpp_action::Client<SimpleAction>::SharedPtr _rgbd_sync_client;
    rclcpp_action::Client<SimpleAction>::SharedPtr _detect_client;
    rclcpp_action::Client<SimpleAction>::SharedPtr _compose_items_client;
    rclcpp_action::Client<SimpleAction>::SharedPtr _estimate_shapes_client;
    rclcpp_action::Client<SimpleAction>::SharedPtr _generate_octomap_client;

    rclcpp_action::Client<SimpleAction>::SendGoalOptions _send_goal_options;

    void goalResponseCallback(std::shared_future<GoalSimpleAction::SharedPtr> future);
    void resultCallback(const GoalSimpleAction::WrappedResult &result);
    void feedbackCallback(GoalSimpleAction::SharedPtr, const std::shared_ptr<const SimpleAction::Feedback> feedback);
};