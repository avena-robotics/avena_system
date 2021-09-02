#ifndef PHYSICS_CLIENT_HANDLER__COMMONS_HPP_
#define PHYSICS_CLIENT_HANDLER__COMMONS_HPP_

// ___CPP___
#include <exception>
#include <Eigen/Dense>

// ___Avena___
#include <bullet_client/b3RobotSimulatorClientAPI.h>
#include <helpers_commons/helpers_commons.hpp>

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
