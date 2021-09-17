#ifndef GENERATE_PATH__COMMONS_HPP_
#define GENERATE_PATH__COMMONS_HPP_

// ___CPP___
#include <vector>
#include <Eigen/Dense>

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// ___Avena___
#include <helpers_commons/helpers_commons.hpp>
#include <helpers_vision/helpers_vision.hpp>
#include <kinematics/kinematics.hpp>
#include <physics_client_handler/physics_client_handler.hpp>
#include <custom_interfaces/msg/generated_path.hpp>
#include <custom_interfaces/msg/end_effector_pose.hpp>

namespace generate_path
{
  using namespace std::chrono_literals;

  using ArmConfiguration = std::vector<double>;
  using Path = std::vector<ArmConfiguration>;
  using Limits = helpers::commons::Limits;

  using GeneratedPath = custom_interfaces::msg::GeneratedPath;
  using EndEffectorPose = custom_interfaces::msg::EndEffectorPose;
  using MovementSequence = std::vector<EndEffectorPose>;

  constexpr int INVALID_HANDLE = -1;

  enum class ReturnCode
  {
    SUCCESS = 0,
    FAILURE
  };

  struct PathPlanningInput
  {
    ArmConfiguration start_state;
    std::vector<ArmConfiguration> goal_states;
    Eigen::Affine3d start_end_effector_pose;
    Eigen::Affine3d goal_end_effector_pose;
    std::vector<Limits> limits;
    size_t num_dof;
    physics_client_handler::PhysicsClientHandler::SharedPtr physics_client_handler;
    physics_client_handler::Obstacles obstacles;
    kinematics::Kinematics::SharedPtr kinematics_engine;
    std::chrono::duration<double> timeout;
  };

  class GeneratePathError : public std::exception
  {
  public:
    explicit GeneratePathError(const std::string &error)
    {
      _error = "[Generate path]: " + error;
    }

    const char *what() const noexcept override
    {
      return _error.c_str();
    }

  private:
    std::string _error;
  };

} // namespace generate_path

#endif // GENERATE_PATH__COMMONS_HPP_
