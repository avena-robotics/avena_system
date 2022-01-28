// CPP
#include <fstream>
#include <filesystem>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <warehouse_ros/database_loader.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/move_group_interface/move_group_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("generate_trajectory_from_waypoints");
static const std::string PLANNING_GROUP = "avena_arm";
static constexpr int DB_CONNECT_TIMEOUT = 20; // seconds

int main(int argc, char **argv)
{
    try
    {
        rclcpp::init(argc, argv);
        rclcpp::NodeOptions node_options;
        node_options.allow_undeclared_parameters(true);
        node_options.automatically_declare_parameters_from_overrides(true);
        rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("generate_trajectory_from_waypoints", node_options);

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        std::thread([&executor]()
                    { executor.spin(); })
            .detach();

        const auto qos_latching = rclcpp::QoS(1).transient_local().reliable();
        auto trajectory_pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/debug/trajectory", qos_latching);

        // std::filesystem::path base_path;
        // node->get_parameter_or("base_path", base_path, std::filesystem::path("/home/avena/Documents"));
        // RCLCPP_INFO_STREAM(LOGGER, "Generated trajectories will be saved to path: " << base_path);

        std::string host;
        node->get_parameter_or("warehouse_host", host, std::string("localhost"));
        RCLCPP_INFO_STREAM(LOGGER, "Warehouse host: " << host);

        int port;
        node->get_parameter_or("warehouse_port", port, 33829);
        RCLCPP_INFO_STREAM(LOGGER, "Warehouse port: " << port);

        warehouse_ros::DatabaseLoader db_loader(node);
        warehouse_ros::DatabaseConnection::Ptr warehouse_connection = db_loader.loadDatabase();
        warehouse_connection->setParams(host, port, DB_CONNECT_TIMEOUT);
        if (!warehouse_connection->connect())
            throw std::runtime_error("Failed to connect to DB");

        RCLCPP_INFO(LOGGER, "Connected to DB successfully");

        // Initialize MoveGroup Interface
        moveit::planning_interface::MoveGroupInterface move_group_interface(node, PLANNING_GROUP);

        // Initialize Warehouse robot state storage and read save states
        auto rss = moveit_warehouse::RobotStateStorage(warehouse_connection);
        std::vector<std::string> names;
        rss.getKnownRobotStates(names);
        for (size_t i = 0; i < names.size() - 1; i++)
        {
            RCLCPP_INFO_STREAM(LOGGER, "Generating trajectory from \"" << names[i] << "\" state to \"" << names[i + 1] << "\" state");
            moveit_warehouse::RobotStateWithMetadata start_rswm;
            if (!rss.getRobotState(start_rswm, names[i]))
                throw std::runtime_error("Cannot read start state. Exiting...");

            moveit_warehouse::RobotStateWithMetadata goal_rswm;
            if (!rss.getRobotState(goal_rswm, names[i + 1]))
                throw std::runtime_error("Cannot read goal state. Exiting...");

            // Set start stare for motion planning
            moveit_msgs::msg::RobotState start_robot_state_msg;
            start_robot_state_msg.joint_state = start_rswm->joint_state;
            move_group_interface.setStartState(start_robot_state_msg);

            // Set goal state for motion planning
            if (!move_group_interface.setJointValueTarget(goal_rswm->joint_state))
                throw std::runtime_error("Cannot set target joint values. Exiting...");

            // Plan
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
                throw std::runtime_error("Planning failed. Exiting...");

            trajectory_pub->publish(plan.trajectory_.joint_trajectory);
            RCLCPP_INFO(LOGGER, "Trajectory published on the topic to be saved");

            success = (move_group_interface.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
                throw std::runtime_error("Execution failed. Exiting...");
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        RCLCPP_ERROR(LOGGER, e.what());
        // rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(LOGGER, "Generating trajectories finished");
    // rclcpp::shutdown();
    return 0;
}