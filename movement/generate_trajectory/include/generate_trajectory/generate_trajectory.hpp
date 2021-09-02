#ifndef GENERATE_TRAJECTORY__GENERATE_TRAJECTORY_HPP_
#define GENERATE_TRAJECTORY__GENERATE_TRAJECTORY_HPP_

// ___CPP___
#include <iomanip>

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// ___Avena___
#include <custom_interfaces/msg/generated_path.hpp>
#include <helpers_commons/helpers_commons.hpp>

// ___Package___
#include "path.h"
#include "trajectory.h"
#include "visibility_control.h"

namespace generate_trajectory
{
  enum class ReturnCode
  {
    SUCCESS = 0,
    FAILURE
  };

  using GeneratedPath = custom_interfaces::msg::GeneratedPath;

  class GenerateTrajectory
  {
  public:
    explicit GenerateTrajectory(rclcpp::Node::SharedPtr node) noexcept(false);
    virtual ~GenerateTrajectory() = default;
    trajectory_msgs::msg::JointTrajectory::SharedPtr generateTrajectoryFromPath(const GeneratedPath::SharedPtr &generated_path);

    using UniquePtr = std::unique_ptr<GenerateTrajectory>;
    using SharedPtr = std::shared_ptr<GenerateTrajectory>;

  private:
    // ___Methods___
    int _generateTrajectoryFromPathSegment(trajectory_msgs::msg::JointTrajectory &segment_trajectory, trajectory_msgs::msg::JointTrajectory &out_trajectory);
    void _generateAcceleration(Eigen::VectorXd maxAcceleration, Trajectory &trajectory);
    void _convertToMsg(Trajectory &trajectory, const std::vector<std::string> &joint_names, trajectory_msgs::msg::JointTrajectory &out_trajectory);
    ReturnCode _getParametersFromServer();
    void _initialize();

    // ___Attributes___
    rclcpp::Node::SharedPtr _node;
    GeneratedPath::SharedPtr _current_generated_path;
    helpers::commons::RobotInfo _robot_info;
    std::vector<std::string> _joint_names;
    double _time_step;
  };

} // namespace generate_trajectory

#endif
