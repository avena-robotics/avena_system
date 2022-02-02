#include "generate_trajectory/generate_trajectory.hpp"

namespace generate_trajectory
{
    GenerateTrajectory::GenerateTrajectory(rclcpp::Node::SharedPtr node)
        : _node(node)
    {
        RCLCPP_INFO(_node->get_logger(), "[Generate trajectory] Initialization of ROS2 trajectory generator.");
        if (_getParametersFromServer() != ReturnCode::SUCCESS)
            throw std::runtime_error("Cannot read parameters from server");
    }

    ReturnCode GenerateTrajectory::_getParametersFromServer()
    {
        RCLCPP_INFO_ONCE(_node->get_logger(), "[Generate trajectory] Reading parameters from the server");

        nlohmann::json parameters = helpers::commons::getParameter("robot");
        if (parameters.empty())
            return ReturnCode::FAILURE;

        if (auto robot_info = helpers::commons::getRobotInfo())
            _robot_info = *robot_info;
        else
            return ReturnCode::FAILURE;

        const double controller_frequency = parameters[_robot_info.robot_name]["controller_frequency"].get<double>();
        RCLCPP_INFO_STREAM(_node->get_logger(), "[Generate trajectory] Trajectories generated for controller working with " << controller_frequency << " Hz");
        _time_step = 1.0 / controller_frequency;

        RCLCPP_INFO(_node->get_logger(), "[Generate trajectory] Parameters read successfully...");
        return ReturnCode::SUCCESS;
    }

    trajectory_msgs::msg::JointTrajectory::SharedPtr GenerateTrajectory::generateTrajectoryFromPath(const GeneratedPath::SharedPtr &generated_path)
    {
        helpers::Timer timer("Generate trajectory: ", _node->get_logger());

        if (!generated_path)
            throw std::runtime_error("Incoming generated path segments is not set");

        RCLCPP_INFO(_node->get_logger(), "[Generate trajectory] Generating trajectory path");

        trajectory_msgs::msg::JointTrajectory::SharedPtr out_full_trajectory = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
        double time_from_start_offset = 0;
        for (auto &path_segment : generated_path->path_segments)
        {
            trajectory_msgs::msg::JointTrajectory trajectory;
            if (_generateTrajectoryFromPathSegment(path_segment, trajectory))
                throw std::runtime_error("Error occured while generating trajectory");

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

        RCLCPP_INFO(_node->get_logger(), "[Generate trajectory] Generate trajectory finished");
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
                RCLCPP_INFO(_node->get_logger(), "[Generate trajectory] Trajectory generation failed.");
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
