#include <generate_trajectory_from_waypoints/generate_trajectory_from_waypoints.hpp>

namespace generate_trajectory_from_waypoints
{
    GenerateTrajectoryFromWaypoints::GenerateTrajectoryFromWaypoints(rclcpp::NodeOptions options)
        : Node("generate_trajectory_from_waypoints", options
                                                         .allow_undeclared_parameters(true)
                                                         .automatically_declare_parameters_from_overrides(true))
    {
        std::string host;
        get_parameter_or("warehouse_host", host, std::string("localhost"));
        RCLCPP_INFO_STREAM(LOGGER, "Warehouse host: " << host);

        int port;
        get_parameter_or("warehouse_port", port, 33829);
        RCLCPP_INFO_STREAM(LOGGER, "Warehouse port: " << port);

        const auto qos_latching = rclcpp::QoS(1).transient_local().reliable();
        _trajectory_pub = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/debug/trajectory", qos_latching);

        _action_server = rclcpp_action::create_server<Action>(
            get_node_base_interface(),
            get_node_clock_interface(),
            get_node_logging_interface(),
            get_node_waitables_interface(),
            "/generate_trajectories",
            std::bind(&GenerateTrajectoryFromWaypoints::_handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&GenerateTrajectoryFromWaypoints::_handleCancel, this, std::placeholders::_1),
            std::bind(&GenerateTrajectoryFromWaypoints::_handleAccepted, this, std::placeholders::_1));

        std::thread([=]()
                    {
                        // Wait a little to make sure that this instance was created
                        std::this_thread::sleep_for(std::chrono::seconds(2));
                        warehouse_ros::DatabaseLoader db_loader(shared_from_this());
                        auto warehouse_connection = db_loader.loadDatabase();
                        warehouse_connection->setParams(host, port, DB_CONNECT_TIMEOUT);
                        if (!warehouse_connection->connect())
                            throw std::runtime_error("Failed to connect to DB");

                        RCLCPP_INFO(LOGGER, "Connected to DB successfully");
                        _robot_state_storage = std::make_unique<moveit_warehouse::RobotStateStorage>(warehouse_connection);
                        _db_connected = true;
                    })
            .detach();
    }

    rclcpp_action::GoalResponse GenerateTrajectoryFromWaypoints::_handleGoal(const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const Action::Goal> /*goal*/)
    {
        RCLCPP_INFO(LOGGER, "Got goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse GenerateTrajectoryFromWaypoints::_handleCancel(const std::shared_ptr<GoalHandleAction> /*goal_handle*/)
    {
        RCLCPP_INFO(LOGGER, "Got request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void GenerateTrajectoryFromWaypoints::_handleAccepted(const std::shared_ptr<GoalHandleAction> goal_handle)
    {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&GenerateTrajectoryFromWaypoints::_generateTrajTrigExecute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void GenerateTrajectoryFromWaypoints::_generateTrajTrigExecute(const std::shared_ptr<GoalHandleAction> goal_handle)
    {
        auto result = std::make_shared<Action::Result>();
        auto feedback = std::make_shared<Action::Feedback>();

        if (!_db_connected)
        {
            result->message = "Connection to Warehouse database was not established.";
            // RCLCPP_ERROR(LOGGER,  message + " Exiting...");
            goal_handle->abort(result);
            return;
        }

        // Initialize MoveGroup Interface
        moveit::planning_interface::MoveGroupInterface move_group_interface(shared_from_this(), PLANNING_GROUP);

        // Initialize Warehouse robot state storage and read save states
        std::vector<std::string> names;
        _robot_state_storage->getKnownRobotStates(names);
        if (names.size() < 2)
        {
            // RCLCPP_ERROR(LOGGER, "There are no robot states save in Warehouse database. Skipping...");
            result->message = "There are no robot states save in Warehouse database.";
            goal_handle->abort(result);
            return;
        }

        int nr_successes = 0;
        for (size_t i = 0; i < names.size() - 1; i++)
        {
            feedback->progress = "Generating trajectory from \"" + names[i] + "\" state to \"" + names[i + 1] + "\" state";
            goal_handle->publish_feedback(feedback);
            
            moveit_warehouse::RobotStateWithMetadata start_rswm;
            if (!_robot_state_storage->getRobotState(start_rswm, names[i]))
            {
                // RCLCPP_ERROR(LOGGER, "Cannot read start robot state from . Skipping...");
                result->message = "Cannot read start robot state from Warehouse database. Exiting...";
                goal_handle->abort(result);
                return;
            }

            moveit_warehouse::RobotStateWithMetadata goal_rswm;
            if (!_robot_state_storage->getRobotState(goal_rswm, names[i + 1]))
            {
                result->message = "Cannot read goal robot state from Warehouse database. Exiting...";
                goal_handle->abort(result);
                return;
            }

            // Set start stare for motion planning
            moveit_msgs::msg::RobotState start_robot_state_msg;
            start_robot_state_msg.joint_state = start_rswm->joint_state;
            move_group_interface.setStartState(start_robot_state_msg);

            // Set goal state for motion planning
            if (!move_group_interface.setJointValueTarget(goal_rswm->joint_state))
            {
                result->message = "Cannot set target joint values. Exiting...";
                goal_handle->abort(result);
                return;
            }

            // Plan
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                feedback->progress = "Planning failed to this pair of waypoints. Trying next pair...";
                RCLCPP_ERROR(LOGGER, feedback->progress);
                goal_handle->publish_feedback(feedback);
                continue;
            }

            nr_successes++;

            _trajectory_pub->publish(plan.trajectory_.joint_trajectory);
            RCLCPP_INFO(LOGGER, "Trajectory published on the topic to be saved");
        }

        result->message = "Successfully generated trajectories for " + std::to_string(nr_successes) + " / " + std::to_string(names.size() - 1) + " paths";
        RCLCPP_INFO(LOGGER, result->message);
        goal_handle->succeed(result);
    }

} // namespace generate_trajectory_from_waypoints

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(generate_trajectory_from_waypoints::GenerateTrajectoryFromWaypoints)
