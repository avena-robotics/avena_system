#ifndef PHYSICS_CLIENT_HANDLER__COMMONS_HPP_
#define PHYSICS_CLIENT_HANDLER__COMMONS_HPP_

// ___CPP___
#include <tuple>
#include <exception>

// ___Other___
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>

// ___Avena___
#include <bullet_client/b3RobotSimulatorClientAPI.h>
#include <helpers_commons/helpers_commons.hpp>
#include <helpers_vision/helpers_vision.hpp>

// ___Package___
#include "physics_client_handler/visibility_control.h"

namespace physics_client_handler
{
  using Obstacles = std::vector<int>;
  using ArmConfiguration = std::vector<double>;
  using Limits = helpers::commons::Limits;
  
  const int INVALID_HANDLE = -1;

  class PhysicsClientHandlerError : public std::exception
  {
  public:
    explicit PhysicsClientHandlerError(const std::string &error)
    {
      _error = "[Physics client handler]: " + error;
    }

    const char *what() const noexcept override
    {
      return _error.c_str();
    }

  private:
    std::string _error;
  };

} // namespace physics_client_handler

#endif // PHYSICS_CLIENT_HANDLER__COMMONS_HPP_
