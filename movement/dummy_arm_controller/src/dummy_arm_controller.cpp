#include "dummy_arm_controller/dummy_arm_controller.hpp"

namespace dummy_arm_controller
{
    DummyArmController::DummyArmController(rclcpp::NodeOptions options)
        : Node("avena_arm_controller", options
                                           .allow_undeclared_parameters(true)
                                           .automatically_declare_parameters_from_overrides(true))
    {
        RCLCPP_INFO(LOGGER, "Initializing dummy joint trajectory controller");

        _action_server = rclcpp_action::create_server<Action>(
            get_node_base_interface(),
            get_node_clock_interface(),
            get_node_logging_interface(),
            get_node_waitables_interface(),
            "avena_arm_controller/follow_joint_trajectory",
            std::bind(&DummyArmController::_handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DummyArmController::_handleCancel, this, std::placeholders::_1),
            std::bind(&DummyArmController::_handleAccepted, this, std::placeholders::_1));

        const auto qos_latching = rclcpp::QoS(1).transient_local().reliable();
        _trajectory_pub = create_publisher<trajectory_msgs::msg::JointTrajectory>("/debug/trajectory", qos_latching);
        _dummy_joint_state_pub = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(10).reliable());
        _current_joint_states.name.clear();

        // TODO: Maybe read this from MoveIt or robot_description???
        for (size_t i = 0; i < 6; i++)
        {
            _current_joint_states.name.push_back("avena_joint_" + std::to_string(i + 1));
            _current_joint_states.position.push_back(0);
            _current_joint_states.velocity.push_back(0);
            _current_joint_states.effort.push_back(0);
        }

        _joint_states_timer = create_wall_timer(
            std::chrono::milliseconds(10), [this]()
            {
                sensor_msgs::msg::JointState joint_states_to_publish;
                {
                    std::lock_guard<std::mutex> lg(_current_joint_states_mtx);
                    joint_states_to_publish = _current_joint_states;
                }
                joint_states_to_publish.header.stamp = get_clock()->now();

                _dummy_joint_state_pub->publish(joint_states_to_publish);
            });
    }

    rclcpp_action::GoalResponse DummyArmController::_handleGoal(const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const Action::Goal> /*goal*/)
    {
        RCLCPP_INFO(LOGGER, "Got goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse DummyArmController::_handleCancel(const std::shared_ptr<GoalHandleAction> /*goal_handle*/)
    {
        RCLCPP_INFO(LOGGER, "Got request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void DummyArmController::_handleAccepted(const std::shared_ptr<GoalHandleAction> goal_handle)
    {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&DummyArmController::_execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void DummyArmController::_execute(const std::shared_ptr<GoalHandleAction> goal_handle)
    {
        RCLCPP_INFO(LOGGER, "Publishing trajectory to topic");
        const auto goal = goal_handle->get_goal();

        // Publish generated trajectory on topic
        _trajectory_pub->publish(goal->trajectory);

        // Set current position of arm to last from trajectory
        {
            std::lock_guard<std::mutex> lg(_current_joint_states_mtx);
            const auto &current_jtp = goal->trajectory.points.back();
            _current_joint_states.position = current_jtp.positions;
            _current_joint_states.velocity = current_jtp.velocities;
            _current_joint_states.effort = current_jtp.effort;
        }

        // Check if goal is done
        if (rclcpp::ok())
        {
            auto result = std::make_shared<Action::Result>();
            goal_handle->succeed(result);
            RCLCPP_INFO_STREAM(LOGGER, "Goal Succeeded");
        }
    }

} // namespace dummy_arm_controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dummy_arm_controller::DummyArmController)
