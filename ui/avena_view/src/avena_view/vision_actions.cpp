#include "avena_view/vision_actions.h"

VisionActions::VisionActions(Ui::AvenaViewWidget *ui_ptr, rclcpp::Node::SharedPtr node_shared_ptr)
    : _ui_ptr(ui_ptr), _node_shared_ptr(node_shared_ptr)
{
    using namespace std::placeholders;

    qRegisterMetaType<std::string>("std::string");

    _rgbd_sync_client = rclcpp_action::create_client<SimpleAction>(_node_shared_ptr, "/rgbd_sync");
    _detect_client = rclcpp_action::create_client<SimpleAction>(_node_shared_ptr, "detect_action");
    _compose_items_client = rclcpp_action::create_client<SimpleAction>(_node_shared_ptr, "compose_items");
    _estimate_shapes_client = rclcpp_action::create_client<SimpleAction>(_node_shared_ptr, "estimate_shape");
    _generate_octomap_client = rclcpp_action::create_client<SimpleAction>(_node_shared_ptr, "octomap_generator");

    _send_goal_options = rclcpp_action::Client<SimpleAction>::SendGoalOptions();
    _send_goal_options.feedback_callback = std::bind(&VisionActions::feedbackCallback, this, _1, _2);
    _send_goal_options.result_callback = std::bind(&VisionActions::resultCallback, this, _1);
    _send_goal_options.goal_response_callback = std::bind(&VisionActions::goalResponseCallback, this, _1);

    connectSlotsToSignals();
}

VisionActions::~VisionActions()
{
}

void VisionActions::executeRgbdSync()
{
    if (!_rgbd_sync_client->wait_for_action_server(1s))
    {
        writeToConsole("rgbd sync server not available after waiting", _ui_ptr->visionAndRobotConsoleLog);
        return;
    }
    auto goal_msg = SimpleAction::Goal();
    _rgbd_sync_client->async_send_goal(goal_msg, _send_goal_options);
}

void VisionActions::executeDetect()
{
    if (!_detect_client->wait_for_action_server(1s))
    {
        writeToConsole("detect server not available after waiting", _ui_ptr->visionAndRobotConsoleLog);
        return;
    }
    auto goal_msg = SimpleAction::Goal();
    _detect_client->async_send_goal(goal_msg, _send_goal_options);
}
void VisionActions::executeComposeItems()
{
    if (!_compose_items_client->wait_for_action_server(1s))
    {
        writeToConsole("compose items server not available after waiting", _ui_ptr->visionAndRobotConsoleLog);
        return;
    }
    auto goal_msg = SimpleAction::Goal();
    _compose_items_client->async_send_goal(goal_msg, _send_goal_options);
}
void VisionActions::executeEstimateShape()
{
    if (!_estimate_shapes_client->wait_for_action_server(1s))
    {
        writeToConsole("estimate shape server not available after waiting", _ui_ptr->visionAndRobotConsoleLog);
        return;
    }
    auto goal_msg = SimpleAction::Goal();
    _estimate_shapes_client->async_send_goal(goal_msg, _send_goal_options);
}
void VisionActions::executeGenerateOctomap()
{
    if (!_generate_octomap_client->wait_for_action_server(1s))
    {
        writeToConsole("generate octomap server not available after waiting", _ui_ptr->visionAndRobotConsoleLog);
        return;
    }
    auto goal_msg = SimpleAction::Goal();
    _generate_octomap_client->async_send_goal(goal_msg, _send_goal_options);
}


void VisionActions::goalResponseCallback([[maybe_unused]] std::shared_future<GoalSimpleAction::SharedPtr> future)
{
    auto goal_handle = future.get();
    if (!goal_handle) {
        emit consoleMsgRequested("goal was rejected");
    } else {
        emit consoleMsgRequested("goal was accepted by server");
    }
}

void VisionActions::resultCallback([[maybe_unused]] const GoalSimpleAction::WrappedResult &result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        emit consoleMsgRequested("goal was succeeded");
        break;
    case rclcpp_action::ResultCode::ABORTED:
        emit consoleMsgRequested("goal was aborted");
        return;
    case rclcpp_action::ResultCode::CANCELED:
        emit consoleMsgRequested("goal was canceled");
        return;
    default:
        emit consoleMsgRequested("unknown error");
        return;
    }
}

void VisionActions::writeConsoleMsg(const std::string& msg)
{
    writeToConsole(msg, _ui_ptr->visionAndRobotConsoleLog);
}

void VisionActions::feedbackCallback([[maybe_unused]] GoalSimpleAction::SharedPtr, [[maybe_unused]] const std::shared_ptr<const SimpleAction::Feedback> feedback)
{
}

void VisionActions::connectSlotsToSignals()
{
    connect(this, SIGNAL(consoleMsgRequested(const std::string&)), this, SLOT(writeConsoleMsg(const std::string&)));

    connect(_ui_ptr->executeRGBDSyncButton, SIGNAL(clicked()), this, SLOT(executeRgbdSync()));
    connect(_ui_ptr->executeDetectButton, SIGNAL(clicked()), this, SLOT(executeDetect()));
    connect(_ui_ptr->executeEstimateShapeButton, SIGNAL(clicked()), this, SLOT(executeEstimateShape()));
    connect(_ui_ptr->executeComposeButton, SIGNAL(clicked()), this, SLOT(executeComposeItems()));
    connect(_ui_ptr->generateOctomapButton, SIGNAL(clicked()), this, SLOT(executeGenerateOctomap()));
}
