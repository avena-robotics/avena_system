#ifndef GENERATE_PATH__GENERATE_PATH_HPP_
#define GENERATE_PATH__GENERATE_PATH_HPP_

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// ___Avena___
#include <custom_interfaces/action/generate_path_pose.hpp>
#include <custom_interfaces/msg/generated_path.hpp>
#include <helpers_commons/helpers_commons.hpp>
#include <bullet_client/b3RobotSimulatorClientAPI.h>

// ___Package___
#include "generate_path/visibility_control.h"

// FIXME:
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> dis(0.0, 1.0);
std::uniform_real_distribution<> joint1_3_5_7(-2.9671, 2.9671);
std::uniform_real_distribution<> joint2(-1.8326, 1.8326);
std::uniform_real_distribution<> joint4(-3.1416, 0);
std::uniform_real_distribution<> joint6(-0.0873, 3.8223);
int END_EFECTOR_LINK_INDEX = 6;

std::vector<float> semi_random_sample(double steer_goal_p, std::vector<float> &q_goal)
{
  double prob = dis(gen);
  std::vector<float> sampled_joints;

  if (prob < steer_goal_p)
  {
    return q_goal;
  }
  else
  {
    sampled_joints.push_back(joint1_3_5_7(gen));
    sampled_joints.push_back(joint2(gen));
    sampled_joints.push_back(joint1_3_5_7(gen));
    sampled_joints.push_back(joint4(gen));
    sampled_joints.push_back(joint1_3_5_7(gen));
    sampled_joints.push_back(joint6(gen));
    sampled_joints.push_back(joint1_3_5_7(gen));

    return sampled_joints;
  }
}

float get_euclidean_distance(std::vector<float> &q1, std::vector<float> &q2)
{
  assert((q1.size() == q2.size()) && "Vectors must be of the same length.");
  float distance = 0;
  int length = q1.size();

  for (int i = 0; i < length; i++)
  {
    distance += (q2[i] - q1[i]) * (q2[i] - q1[i]);
  }

  distance = sqrt(distance);
  return distance;
}

std::vector<float> nearest(std::vector<std::vector<float>> &V, std::vector<float> &q_rand)
{
  double distance = 999999;
  double distance_to_compare;
  std::vector<float> q_nearest;

  int length = V.size();
  for (int i = 0; i < length; i++)
  {
    distance_to_compare = get_euclidean_distance(q_rand, V[i]);
    if (distance_to_compare < distance)
    {
      q_nearest = V[i];
      distance = distance_to_compare;
    }
  }
  return q_nearest;
}

std::vector<float> steer(std::vector<float> &q_nearest, std::vector<float> &q_rand, double delta_q)
{
  std::vector<float> q_new;
  std::vector<float> q_hat;

  if (get_euclidean_distance(q_rand, q_nearest) <= delta_q)
  {
    q_new = q_rand;
  }
  else
  {
    int len = q_rand.size();
    for (int i = 0; i < len; i++)
    {
      q_hat.push_back((q_rand[i] - q_nearest[i]) / get_euclidean_distance(q_rand, q_nearest));
      q_new.push_back(q_nearest[i] + q_hat[i] * delta_q);
    }
  }
  return q_new;
}

bool check_collision_free(b3RobotSimulatorClientAPI_NoGUI *env, std::vector<float> &q_new, double distance, b3RobotSimulatorGetContactPointsArgs &collision_args, std::vector<int> &obstacles, std::vector<int> &robot,
                          int collision_threshold)
{

  int obs = 0;
  int robot_id = robot[0];
  for (int i = 0; i <= END_EFECTOR_LINK_INDEX; i++)
  {
    env->resetJointState(robot_id, i, q_new[i]);
  }
  // env->stepSimulation();
  //! Distanced obstacles
  b3ContactInformation info;
  int robot_len = robot.size();
  int obstacles_len = obstacles.size();
  for (int i = 0; i < robot_len; i++)
  {
    collision_args.m_bodyUniqueIdA = robot[i];
    for (int j = 0; j < obstacles_len; j++)
    {
      collision_args.m_bodyUniqueIdB = obstacles[j];
      env->getClosestPoints(collision_args, distance, &info);
      obs += info.m_numContactPoints;
    }
  }

  //! Contant obstacles (robot with gripper)
  // for (int i = 0; i < robot_len - 1; i++)
  // {
  // 	collision_args.m_bodyUniqueIdA = robot[i];
  // 	for (int j = i + 1; j < robot_len; j++)
  // 	{
  // 		collision_args.m_bodyUniqueIdB = robot[j];
  // 		env->getContactPoints(collision_args, &info);
  // 		obs += info.m_numContactPoints;
  // 	}
  // }
  // std::cout<<obs<<std::endl;
  if (obs > collision_threshold)
    return false;
  else
    return true;
}

std::vector<std::vector<float>> rrt(std::vector<float> &q_init, std::vector<float> &q_goal, int MAX_ITERS, double delta_q, double steer_goal_p, b3RobotSimulatorClientAPI_NoGUI *env, double distance,
                                    std::vector<int> &obstacles, std::vector<int> &robot, int collision_threshold)
{
  /*
    :param q_init: initial configuration
    :param q_goal: goal configuration
    :param MAX_ITERS: max number of iterations
    :param delta_q: steer distance
    :param steer_goal_p: probability of steering towards the goal
    :param distance: threshold distance for check_collision
    :returns path: series of joint angles
    */
  std::vector<std::vector<float>> V = {q_init};
  std::vector<std::vector<std::vector<float>>> E;
  std::vector<std::vector<float>> path;
  bool found = false;

  std::vector<float> q_rand;
  std::vector<float> q_nearest;
  std::vector<float> q_new;

  b3RobotSimulatorGetContactPointsArgs COLLISION_ARGS;

  for (int i = 0; i < MAX_ITERS; i++)
  {
    q_rand = semi_random_sample(steer_goal_p, q_goal);
    q_nearest = nearest(V, q_rand);
    q_new = steer(q_nearest, q_rand, delta_q);
    if (check_collision_free(env, q_new, distance, COLLISION_ARGS, obstacles, robot, collision_threshold))
    {
      auto result1 = std::find(begin(V), end(V), q_new);
      if (result1 == std::end(V))
      {
        V.push_back(q_new);
      }
      std::vector<std::vector<float>> edge = {q_nearest, q_new};
      auto result2 = std::find(begin(E), end(E), edge);
      if (result2 == std::end(E))
      {
        E.push_back(edge);
      }
      if (get_euclidean_distance(q_goal, q_new) < delta_q)
      {
        V.push_back(q_goal);
        std::vector<std::vector<float>> last_edge = {q_nearest, q_goal};
        E.push_back(last_edge);
        found = true;
        break;
      }
    }
  }
  if (found)
  {
    std::cout << "FOUND PATH" << std::endl;
    std::vector<float> current_q = q_goal;
    path.push_back(current_q);
    while (current_q != q_init)
    {
      // std::cout<<get_euclidean_distance(current_q, q_init)<<std::endl;
      int len = E.size();
      for (int i = 0; i < len; i++)
      {
        if (E[i][1] == current_q)
        {
          current_q = E[i][0];
          path.push_back(current_q);
        }
      }
    }
    std::reverse(path.begin(), path.end());
  }
  return path;
}

namespace generate_path
{
  enum class ReturnCode
  {
    SUCCESS = 0,
    FAILURE
  };

  class GeneratePath : public rclcpp::Node, public helpers::WatchdogInterface
  {
  public:
    using GeneratePathPose = custom_interfaces::action::GeneratePathPose;
    using GoalHandleGeneratePathPose = rclcpp_action::ServerGoalHandle<GeneratePathPose>;
    using ArmConfiguration = std::vector<float>;

    explicit GeneratePath(const rclcpp::NodeOptions &options);
    virtual ~GeneratePath();
    virtual void initNode() override;
    virtual void shutDownNode() override;

  private:
    // ___Methods___
    // ___Go to end effector pose___
    rclcpp_action::GoalResponse _handleGoalPose(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GeneratePathPose::Goal> goal);
    rclcpp_action::CancelResponse _handleCancelPose(const std::shared_ptr<GoalHandleGeneratePathPose> goal_handle);
    void _handleAcceptedPose(const std::shared_ptr<GoalHandleGeneratePathPose> goal_handle);
    void _executePose(const std::shared_ptr<GoalHandleGeneratePathPose> goal_handle);
    ReturnCode _initialize();
    ReturnCode _shutdown();
    ReturnCode _setCurrentJointStatesOnPhysicsServer(const sensor_msgs::msg::JointState::SharedPtr &joint_states);
    ReturnCode _getParametersFromServer();
    void _convertPathSegmentToTrajectoryMsg(const std::vector<ArmConfiguration> &path, trajectory_msgs::msg::JointTrajectory &path_segment);

    // ___Attributes___
    helpers::Watchdog::SharedPtr _watchdog;
    rclcpp_action::Server<GeneratePathPose>::SharedPtr _action_server_pose;
    rclcpp::Publisher<custom_interfaces::msg::GeneratedPath>::SharedPtr _generated_path_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_state_sub;
    sensor_msgs::msg::JointState::SharedPtr _current_joint_states;
    std::shared_ptr<bullet_client::b3RobotSimulatorClientAPI> _bullet_client;
    helpers::commons::RobotInfo _robot_info;

    int _table_idx;
    int _robot_idx;
  };

} // namespace generate_path

#endif // GENERATE_PATH__GENERATE_PATH_HPP_
