#ifndef HELPERS_COMMONS__GEOMETRY_HPP_
#define HELPERS_COMMONS__GEOMETRY_HPP_

// ___CPP___
#include <array>
#include <Eigen/Dense>

// ___Package___
#include "helpers_commons/visibility_control.h"

namespace helpers
{
  namespace commons
  {
    /**
     * @brief Get the Axes Diff Between Quaternions object
     * Calculates per axis difference between quaternions in radians
     * 
     * @tparam T 
     * @param quat0 
     * @param quat1 
     * @return std::array<double, 3> x, y, z axes angles difference in radians
     */
    template <typename T>
    std::array<double, 3> getAxesDiffBetweenQuaternions(const Eigen::Quaternion<T> &quat0, const Eigen::Quaternion<T> &quat1)
    {
      std::array<double, 3> axis_angle_diff;

      for (int i = 0; i < 3; i++)
      {
        auto rot_matrix0 = quat0.normalized().toRotationMatrix();
        auto axis0 = rot_matrix0.col(i);
        auto rot_matrix1 = quat1.normalized().toRotationMatrix();
        auto axis1 = rot_matrix1.col(i);
        double angle = std::atan2(axis0.cross(axis1).norm(), axis0.dot(axis1));
        axis_angle_diff[i] = angle;
      }

      return axis_angle_diff;
    }

    /**
     * @brief Get the Diff Between Affines object
     * Return difference between two affine transforms. Calculates
     * cartesian distance between positions and per axis difference 
     * between quaternions
     * 
     * @tparam T 
     * @param aff0 
     * @param aff1 
     * @return std::array<double, 4> First element is position distance in meters, 
     * remaining 3 are x, y, z axes angles difference in radians 
     */
    template <typename T>
    std::array<double, 4> getDiffBetweenAffines(const Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> &aff0,
                                                const Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> &aff1)
    {
      std::array<double, 4> diff;

      auto position0(aff0.translation());
      Eigen::Quaternion<T> orientation0(aff0.rotation());

      auto position1(aff1.translation());
      Eigen::Quaternion<T> orientation1(aff1.rotation());

      diff[0] = (position0 - position1).norm();
      auto axis_diffs = helpers::commons::getAxesDiffBetweenQuaternions(orientation0, orientation1);
      diff[1] = axis_diffs[0];
      diff[2] = axis_diffs[1];
      diff[3] = axis_diffs[2];

      return diff;
    }

  } // namespace commons

} // namespace helpers

#endif // HELPERS_COMMONS__GEOMETRY_HPP_
