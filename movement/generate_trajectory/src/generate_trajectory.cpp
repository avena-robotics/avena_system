#include "generate_trajectory.h"

namespace generate_trajectory
{
    GenerateTrajectory::GenerateTrajectory(const rclcpp::NodeOptions &options)
        : Node("generate_trajectory_action", options),
          _parameters_read(false)
    {
        RCLCPP_INFO(get_logger(), "Initialization of ROS2 trajectory generator.");
        rclcpp::QoS qos_latching = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        auto getPathCallback = [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr path_msg) -> void
        {
            if (path_msg != nullptr)
                _path_data = path_msg;
            else
                RCLCPP_INFO(get_logger(), "Path topic is empty.");
        };

        _sub_path = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("generated_path", qos_latching, getPathCallback);
        _pub_trajectory = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("trajectory", qos_latching);

        _action_server_generate = rclcpp_action::create_server<custom_interfaces::action::SimpleAction>(this, "generate_trajectory",
                                                                                                        std::bind(&GenerateTrajectory::_handleGoal, this, std::placeholders::_1, std::placeholders::_2),
                                                                                                        std::bind(&GenerateTrajectory::_handleCancel, this, std::placeholders::_1),
                                                                                                        std::bind(&GenerateTrajectory::_handleAccepted, this, std::placeholders::_1));
        _getParametersFromServer();
    }

    int GenerateTrajectory::_getParametersFromServer()
    {
        RCLCPP_INFO_ONCE(get_logger(), "Reading parameters from the server");

        if (_parameters_read)
            return 0;

        nlohmann::json parameters = helpers::commons::getParameter("robot");
        if (parameters.empty())
            return 1;

        const std::string working_side = parameters["working_side"];
        _robot_info = helpers::commons::getRobotInfo(working_side);

        _parameters_read = true;
        RCLCPP_INFO(get_logger(), "Parameters read successfully...");
        return 0;
    }

    rclcpp_action::GoalResponse GenerateTrajectory::_handleGoal(const rclcpp_action::GoalUUID &/*uuid*/, std::shared_ptr<const GenerateTraj::Goal> /*goal*/)
    {
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse GenerateTrajectory::_handleCancel(const std::shared_ptr<GoalHandleGenerateTraj> /*goal_handle*/)
    {
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void GenerateTrajectory::_handleAccepted(const std::shared_ptr<GoalHandleGenerateTraj> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted");
        _execute(goal_handle);
    }

    void GenerateTrajectory::_execute(const std::shared_ptr<GoalHandleGenerateTraj> goal_handle)
    {
        auto result = std::make_shared<GenerateTraj::Result>();

        if (_getParametersFromServer())
        {
            RCLCPP_WARN(get_logger(), "Error occured when reading parameters from server. Aborting...");
            goal_handle->abort(result);
            return;
        }

        if (!generateTrajectoryFromPath())
        {
            RCLCPP_WARN(get_logger(), "Error occured when generating trajectory. Aborting...");
            goal_handle->abort(result);
            return;
        }

        if (rclcpp::ok())
        {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }

        RCLCPP_INFO(this->get_logger(), "finished");
    }

    bool GenerateTrajectory::generateTrajectoryFromPath()
    {
        const float limit = 0.5;
        Eigen::VectorXd maxAcceleration = Eigen::VectorXd::Ones(_robot_info.nr_joints) * limit;
        Eigen::VectorXd maxVelocity = Eigen::VectorXd::Ones(_robot_info.nr_joints) * limit;

        std::list<Eigen::VectorXd> waypoints;
        Eigen::VectorXd waypoint(_robot_info.nr_joints);

        for (size_t i = 0; i < _path_data->points.size(); i++)
        {
            for (size_t j = 0; j < _robot_info.nr_joints; j++)
                waypoint[j] = _path_data->points[i].positions[j];

            waypoints.push_back(waypoint);
        }
        Path converted_path(waypoints, 0.01);
        if (converted_path.getLength() != 0)
        {
            Trajectory trajectory(converted_path, maxVelocity, maxAcceleration, _robot_info.nr_joints, _time_step);
            _generateAcceleration(maxAcceleration, trajectory);
            if (trajectory.isValid())
            {
                _convertToMsgAndPublishTraj(trajectory);
                return true;
            }
            else
            {
                return false;

                RCLCPP_INFO(this->get_logger(), "Trajectory generation failed.");
            }
        }
        else
            _pub_trajectory->publish(*_path_data);
        return true;
    }

    void GenerateTrajectory::_generateAcceleration(Eigen::VectorXd /*maxAcceleration*/, Trajectory &trajectory)
    {
        std::vector<std::vector<double>> acceleration;
        double fulltime = trajectory.getDuration() / _time_step;
        int time_check = trajectory.getDuration() / _time_step;

        if (fulltime - time_check > 0)
            time_check += 1;

        double time = 0;
        for (int j = 0; j <= time_check; j++)
        {
            std::vector<double> accel_values(_robot_info.nr_joints);
            for (size_t i = 0; i < _robot_info.nr_joints; i++)
            {
                if (time < trajectory.getDuration())
                    accel_values[i] = (trajectory.getVelocity(time + _time_step)[i] - trajectory.getVelocity(time)[i]) / _time_step;
                else
                    accel_values[i] = 0.0;
            }
            time += _time_step;
            acceleration.push_back(accel_values);
        }
        trajectory.setAcceleration = acceleration;
    }

    void GenerateTrajectory::_convertToMsgAndPublishTraj(Trajectory &trajectory)
    {
        double fulltime = trajectory.getDuration() / _time_step;
        size_t trajectory_points = trajectory.getDuration() / _time_step;
        trajectory_msgs::msg::JointTrajectoryPoint trajectory_point;
        trajectory_msgs::msg::JointTrajectory trajectory_data;

        trajectory_point.positions.resize(_robot_info.nr_joints);
        trajectory_point.velocities.resize(_robot_info.nr_joints);
        trajectory_point.accelerations.resize(_robot_info.nr_joints);

        std::vector<std::string> joint_names = _path_data->joint_names;

        trajectory_data.joint_names = joint_names;

        if ((fulltime - trajectory_points) > 0)
            trajectory_points += 1;

        double time = 0;

        for (uint32_t point = 0; point <= trajectory_points; point++)
        {
            if (point == trajectory_points)
                time = trajectory.getDuration();

            for (size_t i = 0; i < _robot_info.nr_joints; i++)
            {
                trajectory_point.positions[i] = trajectory.getPosition(time)[i];
                trajectory_point.velocities[i] = trajectory.getVelocity(time)[i];
                trajectory_point.accelerations[i] = trajectory.getAcceleration(time)[i];
            }
            trajectory_point.time_from_start = rclcpp::Duration::from_seconds(time);
            trajectory_data.points.push_back(trajectory_point);

            time += _time_step;
        }
        trajectory_data.header.stamp = this->now();
        _pub_trajectory->publish(trajectory_data);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(generate_trajectory::GenerateTrajectory)
