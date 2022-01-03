#ifndef GENERATE_PATH__L_O_PLANNER_HPP_
#define GENERATE_PATH__L_O_PLANNER_HPP_

// ___Eigen___
#include <Eigen/Geometry>

// ___OMPL___
#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

// ___Package___
#include "generate_path/planners/base_planner.hpp"
#include "generate_path/planners/linear_planner.hpp"
#include "generate_path/planners/orientation_planner.hpp"


namespace generate_path
{
  /***************************
   * Bounds
   * ************************/
  // class Bounds
  // {
  // public:
  //   Bounds();
  //   Bounds(const std::vector<double> &lower, const std::vector<double> &upper);
  //   /** \brief Distance to region inside bounds
  //    *
  //    * Distance of a given value outside the bounds, zero inside the bounds.
  //    * Creates a penalty function that looks like this:
  //    *
  //    * (penalty) ^
  //    *           | \         /
  //    *           |  \       /
  //    *           |   \_____/
  //    *           |----------------> (variable to be constrained)
  //    * */
  //   Eigen::VectorXd penalty(const Eigen::Ref<const Eigen::VectorXd> &x) const;

  //   /** \brief Derivative of the penalty function
  //    * ^
  //    * |
  //    * | -1-1-1 0 0 0 +1+1+1
  //    * |------------------------>
  //    * **/
  //   Eigen::VectorXd derivative(const Eigen::Ref<const Eigen::VectorXd> &x) const;

  //   std::size_t size() const;

  // private:
  //   std::vector<double> lower_, upper_;
  //   std::size_t size_;

  //   friend std::ostream &operator<<(std::ostream &os, const Bounds &bounds);
  // };

  // /** \brief Pretty printing of bounds. **/
  // std::ostream &operator<<(std::ostream &os, const Bounds &bounds);

  /***************************
   * LOConstraint
   * ************************/
  class LOConstraint : public ob::Constraint
  {
  public:
    LOConstraint(const PathPlanningInput &path_planning_input, const rclcpp::Logger &logger);
    virtual ~LOConstraint() = default;
    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;
    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override;

  private:
    Eigen::VectorXd _calcError(const Eigen::Ref<const Eigen::VectorXd> &x) const;
    // Eigen::MatrixXd _calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd> &x) const;
    Eigen::Affine3d _forwardKinematics(const Eigen::Ref<const Eigen::VectorXd> &joint_values) const;
    Eigen::MatrixXd _robotGeometricJacobian(const Eigen::Ref<const Eigen::VectorXd> &joint_values) const;

    PathPlanningInput _path_planning_input;
    rclcpp::Logger _logger;
    Eigen::Vector3d _target_position, _target_orientation;
    Eigen::Quaterniond _target_position_orientation;
    Bounds _bounds;

  };


  /***************************
   * LOPlanner
   * ************************/
  class LOPlanner : public IPlanner
  {
  public:
    LOPlanner(const rclcpp::Logger &logger);
    virtual ~LOPlanner() = default;
    virtual ReturnCode solve(const PathPlanningInput &path_planning_input, std::vector<ArmConfiguration> &out_path) override;

  protected:
    virtual std::vector<ArmConfiguration> _convertOmplStatesToArmConfigurations(ompl::geometric::PathGeometric &ompl_path, size_t num_dof) override;

  private:
    /**
     * @brief Minimum allowed distance between start end effector position
     * and goal end effector position. It is used for IK sampling.
     * Currently not used because OMPL sampling is used
     */
    static constexpr double MIN_DISTANCE = 0.05; // in meters
  };

} // namespace generate_path

#endif // GENERATE_PATH__L_O_PLANNER_HPP_
