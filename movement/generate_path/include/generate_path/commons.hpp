#ifndef GENERATE_PATH__COMMONS_HPP_
#define GENERATE_PATH__COMMONS_HPP_

// ___CPP___
#include <vector>
#include <Eigen/Dense>

// ___Avena___
#include <bullet_client/b3RobotSimulatorClientAPI.h>
#include <helpers_commons/helpers_commons.hpp>


namespace generate_path
{
  using ArmConfiguration = std::vector<double>;
  using Path = std::vector<ArmConfiguration>;
  using Limits = helpers::commons::Limits;

  enum class ReturnCode
  {
    SUCCESS = 0,
    FAILURE
  };

  struct SceneInfo
  {
    bullet_client::b3RobotSimulatorClientAPI::SharedPtr bullet_client;
    int robot_idx;
    int end_effector_idx;
    std::vector<int> joint_handles;

    using SharedPtr = std::shared_ptr<SceneInfo>;
  };

  struct Constraints
  {
    std::vector<Limits> limits;
    std::vector<int> obstacles;
    double safety_distance;
    int contact_number_allowed;

    using SharedPtr = std::shared_ptr<Constraints>;
  };

  struct PathPlanningInput
  {
    SceneInfo::SharedPtr scene_info;
    Constraints::SharedPtr constraints;
    ArmConfiguration start_state;
    ArmConfiguration goal_state;
    Eigen::Affine3d goal_end_effector_pose;
  };

} // namespace generate_path

#endif // GENERATE_PATH__COMMONS_HPP_
