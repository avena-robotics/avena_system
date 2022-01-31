// CPP
#include <chrono>
#include <cinttypes>
#include <memory>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Avena
#include <custom_interfaces/action/trigger.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("generate_trajectory_from_waypoints_trigger");

using Action = custom_interfaces::action::Trigger;

// void feedback_callback(
//     rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr,
//     const std::shared_ptr<const Fibonacci::Feedback> feedback)
// {
//     RCLCPP_INFO(
//         LOGGER,
//         "Next number in sequence received: %" PRId32,
//         feedback->sequence.back());
// }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("generate_trajectories_trigger");
    auto action_client = rclcpp_action::create_client<Action>(node, "/generate_trajectories");

    if (!action_client->wait_for_action_server(std::chrono::seconds(20)))
    {
        RCLCPP_ERROR(LOGGER, "Action server not available after waiting");
        return 1;
    }

    // Populate a goal
    auto goal_msg = Action::Goal();
    RCLCPP_INFO(LOGGER, "Sending goal");
    // Ask server to achieve some goal and wait until it's accepted
    auto send_goal_options = rclcpp_action::Client<Action>::SendGoalOptions();
    send_goal_options.feedback_callback = [=](rclcpp_action::ClientGoalHandle<Action>::SharedPtr,
                                              const std::shared_ptr<const Action::Feedback> feedback) 
                                            {
                                                RCLCPP_INFO(LOGGER, "Feedback: " + feedback->progress);
                                            };
    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

    if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(LOGGER, "send goal call failed :(");
        return 1;
    }

    rclcpp_action::ClientGoalHandle<Action>::SharedPtr goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
        return 1;
    }

    // Wait for the server to be done with the goal
    auto result_future = action_client->async_get_result(goal_handle);

    RCLCPP_INFO(LOGGER, "Waiting for result");
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(LOGGER, "get result call failed :(");
        return 1;
    }

    rclcpp_action::ClientGoalHandle<Action>::WrappedResult wrapped_result = result_future.get();

    switch (wrapped_result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(LOGGER, "Goal was aborted");
        return 1;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(LOGGER, "Goal was canceled");
        return 1;
    default:
        RCLCPP_ERROR(LOGGER, "Unknown result code");
        return 1;
    }

    RCLCPP_INFO(LOGGER, wrapped_result.result->message);

    action_client.reset();
    node.reset();
    rclcpp::shutdown();
    return 0;
}
