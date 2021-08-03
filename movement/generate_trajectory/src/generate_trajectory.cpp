#include "generate_trajectory.h"

#include <iomanip>

namespace generate_trajectory
{
    GenerateTrajectory::GenerateTrajectory(const rclcpp::NodeOptions &options)
        : Node("generate_trajectory", options)
    {
        status = custom_interfaces::msg::Heartbeat::STOPPED;
        _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
    }

    GenerateTrajectory::~GenerateTrajectory()
    {
        shutDownNode();
    }

    void GenerateTrajectory::initNode()
    {
        RCLCPP_DEBUG(get_logger(), "Initializing node");
        status = custom_interfaces::msg::Heartbeat::STARTING;
        if (_initialize() != ReturnCode::SUCCESS)
        {
            RCLCPP_WARN(get_logger(), "Error occured while initializing node");
            return;
        }
        status = custom_interfaces::msg::Heartbeat::RUNNING;
    }

    void GenerateTrajectory::shutDownNode()
    {
        RCLCPP_DEBUG(get_logger(), "Shutting down node");
        status = custom_interfaces::msg::Heartbeat::STOPPING;
        if (_shutdown() != ReturnCode::SUCCESS)
            RCLCPP_ERROR(get_logger(), "Error occured when shutting down node");
        status = custom_interfaces::msg::Heartbeat::STOPPED;
    }

    ReturnCode GenerateTrajectory::_initialize()
    {
        RCLCPP_INFO(get_logger(), "Initialization of ROS2 trajectory generator.");
        rclcpp::QoS qos_latching = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        _generated_path_sub = create_subscription<GeneratedPath>("generated_path", qos_latching,
                                                                 [this](const GeneratedPath::SharedPtr current_generated_path) -> void
                                                                 {
                                                                     std::lock_guard<std::mutex> lg(_current_generated_path_mtx);
                                                                     _current_generated_path = current_generated_path;
                                                                 });
        _trajectory_pub = create_publisher<trajectory_msgs::msg::JointTrajectory>("trajectory", qos_latching);
        _action_server_generate = rclcpp_action::create_server<custom_interfaces::action::SimpleAction>(
            this, "generate_trajectory",
            std::bind(&GenerateTrajectory::_handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&GenerateTrajectory::_handleCancel, this, std::placeholders::_1),
            std::bind(&GenerateTrajectory::_handleAccepted, this, std::placeholders::_1));

        if (_getParametersFromServer() != ReturnCode::SUCCESS)
        {
            RCLCPP_WARN(get_logger(), "Cannot read parameters from server");
            return ReturnCode::FAILURE;
        }

        return ReturnCode::SUCCESS;
    }

    ReturnCode GenerateTrajectory::_shutdown()
    {
        _action_server_generate.reset();
        _generated_path_sub.reset();
        _trajectory_pub.reset();
        return ReturnCode::SUCCESS;
    }

    ReturnCode GenerateTrajectory::_getParametersFromServer()
    {
        RCLCPP_INFO_ONCE(get_logger(), "Reading parameters from the server");

        nlohmann::json parameters = helpers::commons::getParameter("robot");
        if (parameters.empty())
            return ReturnCode::FAILURE;

        const std::string working_side = parameters["working_side"];
        _robot_info = helpers::commons::getRobotInfo(working_side);

        const double controller_frequency = parameters["controller_frequency"].get<double>();
        RCLCPP_INFO_STREAM(get_logger(), "Trajectories generated for controller working with " << controller_frequency << " Hz");
        _time_step = 1.0 / controller_frequency;

        RCLCPP_INFO(get_logger(), "Parameters read successfully...");
        return ReturnCode::SUCCESS;
    }

    rclcpp_action::GoalResponse GenerateTrajectory::_handleGoal(const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const GenerateTrajectoryAction::Goal> /*goal*/)
    {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse GenerateTrajectory::_handleCancel(const std::shared_ptr<GoalHandleGenerateTrajectory> /*goal_handle*/)
    {
        RCLCPP_INFO(get_logger(), "Goal canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void GenerateTrajectory::_handleAccepted(const std::shared_ptr<GoalHandleGenerateTrajectory> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Goal accepted");
        std::thread(std::bind(&GenerateTrajectory::_execute, this, std::placeholders::_1), goal_handle).detach();
    }

    void GenerateTrajectory::_execute(const std::shared_ptr<GoalHandleGenerateTrajectory> goal_handle)
    {
        auto result = std::make_shared<GenerateTrajectoryAction::Result>();

        // Save current state of joints to prevent data races
        GeneratedPath::SharedPtr current_generated_path;
        {
            std::lock_guard<std::mutex> lg(_current_generated_path_mtx);
            if (!_current_generated_path)
            {
                RCLCPP_ERROR(get_logger(), "There is no generated path data coming to module. Aborting...");
                _trajectory_pub->publish(trajectory_msgs::msg::JointTrajectory());
                goal_handle->abort(result);
                return;
            }
            current_generated_path = std::make_shared<GeneratedPath>(*_current_generated_path);
            _current_generated_path.reset();
        }

        if (current_generated_path->path_segments.size() == 0)
        {
            RCLCPP_WARN(get_logger(), "Empty generated path message. Aborting...");
            goal_handle->abort(result);
            return;
        }

        auto generated_trajectory = _generateTrajectoryFromPath(current_generated_path);
        if (!generated_trajectory)
        {
            RCLCPP_WARN(get_logger(), "Error occured when generating trajectory. Aborting...");
            goal_handle->abort(result);
            return;
        }

        generated_trajectory->header.stamp = now();

        // for (size_t i = 0; i < generated_trajectory->points.size() - 1; i++)
        // {
        //     auto ts = static_cast<double>(generated_trajectory->points[i].time_from_start.sec) + generated_trajectory->points[i].time_from_start.nanosec / 1e9;
        //     auto ts_next = static_cast<double>(generated_trajectory->points[i + 1].time_from_start.sec) + generated_trajectory->points[i + 1].time_from_start.nanosec / 1e9;
        //     auto diff = ts_next - ts;
        //     if (std::abs(diff - _time_step) > 0.01)
        //         RCLCPP_WARN_STREAM(get_logger(), "Time step different");
        //     RCLCPP_INFO_STREAM(get_logger(), "i: " << std::setw(5) << i << ": " << std::setw(10) << ts << ", diff: " << std::setw(10) << diff);
        // }

        _trajectory_pub->publish(std::move(generated_trajectory));
        if (rclcpp::ok())
        {
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal succeeded");
        }
    }

    std::unique_ptr<trajectory_msgs::msg::JointTrajectory> GenerateTrajectory::_generateTrajectoryFromPath(const GeneratedPath::SharedPtr &generated_path)
    {
        trajectory_msgs::msg::JointTrajectory::UniquePtr out_full_trajectory(new trajectory_msgs::msg::JointTrajectory);
        double time_from_start_offset = 0;
        for (auto &path_segment : generated_path->path_segments)
        {
            trajectory_msgs::msg::JointTrajectory trajectory;
            _generateTrajectoryFromPathSegment(path_segment, trajectory);

            // Offset time_from_start
            std::for_each(trajectory.points.begin(), trajectory.points.end(),
                          [&time_from_start_offset](trajectory_msgs::msg::JointTrajectoryPoint &tp) mutable
                          {
                              auto time_from_start_sec = static_cast<double>(tp.time_from_start.sec) + tp.time_from_start.nanosec / 1e9;
                              time_from_start_sec += time_from_start_offset;
                              tp.time_from_start = rclcpp::Duration::from_seconds(time_from_start_sec);
                          });

            const auto last_time_from_start = trajectory.points.back().time_from_start;
            time_from_start_offset = static_cast<double>(last_time_from_start.sec) + last_time_from_start.nanosec / 1e9;
            time_from_start_offset += _time_step;
            out_full_trajectory->joint_names = trajectory.joint_names;
            out_full_trajectory->points.insert(out_full_trajectory->points.end(), trajectory.points.begin(), trajectory.points.end());
        }
        return out_full_trajectory;
    }

    int GenerateTrajectory::_generateTrajectoryFromPathSegment(trajectory_msgs::msg::JointTrajectory &segment_trajectory, trajectory_msgs::msg::JointTrajectory &out_trajectory)
    {
        const float limit = 0.5;
        Eigen::VectorXd maxAcceleration = Eigen::VectorXd::Ones(_robot_info.nr_joints) * limit;
        Eigen::VectorXd maxVelocity = Eigen::VectorXd::Ones(_robot_info.nr_joints) * limit;

        std::list<Eigen::VectorXd> waypoints;
        Eigen::VectorXd waypoint(_robot_info.nr_joints);
        _joint_names = segment_trajectory.joint_names;

        for (size_t i = 0; i < segment_trajectory.points.size(); i++)
        {
            for (size_t j = 0; j < _robot_info.nr_joints; j++)
                waypoint[j] = segment_trajectory.points[i].positions[j];

            waypoints.push_back(waypoint);
        }
        Path converted_path(waypoints, 0.01);
        if (converted_path.getLength() != 0)
        {
            Trajectory trajectory(converted_path, maxVelocity, maxAcceleration, _robot_info.nr_joints, _time_step);
            _generateAcceleration(maxAcceleration, trajectory);
            if (trajectory.isValid())
            {
                _convertToMsg(trajectory, _joint_names, out_trajectory);
                return 0;
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Trajectory generation failed.");
                return 1;
            }
        }
        return 0;
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

    void GenerateTrajectory::_convertToMsg(Trajectory &trajectory, const std::vector<std::string> &joint_names, trajectory_msgs::msg::JointTrajectory &out_trajectory)
    {
        double fulltime = trajectory.getDuration() / _time_step;
        size_t trajectory_points = trajectory.getDuration() / _time_step;
        trajectory_msgs::msg::JointTrajectoryPoint trajectory_point;
        // trajectory_msgs::msg::JointTrajectory trajectory_data;

        trajectory_point.positions.resize(_robot_info.nr_joints);
        trajectory_point.velocities.resize(_robot_info.nr_joints);
        trajectory_point.accelerations.resize(_robot_info.nr_joints);

        out_trajectory.joint_names = joint_names;

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
            out_trajectory.points.push_back(trajectory_point);
            time += _time_step;
        }
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(generate_trajectory::GenerateTrajectory)
