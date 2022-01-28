#ifndef SAVE_TRAJECTORY__SAVE_TRAJECTORY_HPP
#define SAVE_TRAJECTORY__SAVE_TRAJECTORY_HPP

// CPP
#include <memory>
#include <fstream>
#include <filesystem>

// ROS2
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

constexpr double JOINT_POSMAX = M_PI;
constexpr double JOINT_VELMAX = 2 * M_PI;
constexpr double JOINT_ACCMAX = 4 * M_PI;
constexpr double JOINT_TORQUEMAX = 256.0;
constexpr double MAX_INT16 = 32767.0;

namespace save_trajectory
{
  static const rclcpp::Logger LOGGER = rclcpp::get_logger("avena_arm_saving_trajectory");

  class SaveTrajectory : public rclcpp::Node
  {
  public:
    explicit SaveTrajectory(rclcpp::NodeOptions options);
    ~SaveTrajectory() = default;

  private:
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr _trajectory_sub;
    std::filesystem::path _base_path;
    void _trajectoryCallback(trajectory_msgs::msg::JointTrajectory::ConstSharedPtr trajectory);

  };

} // namespace save_trajectory

#endif // SAVE_TRAJECTORY__SAVE_TRAJECTORY_HPP
