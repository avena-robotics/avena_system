#ifndef INVERSE_KINEMATICS__COMMONS_HPP_
#define INVERSE_KINEMATICS__COMMONS_HPP_

// ___CPP___
#include <exception>

// ___ROS___
#include <rclcpp/rclcpp.hpp>

// ___Avena___
#include <helpers_commons/helpers_commons.hpp>

namespace inverse_kinematics
{
  // const static rclcpp::Logger LOGGER = rclcpp::get_logger("inverse_kinematics");
  using ArmConfiguration = std::vector<double>;
  using Limits = helpers::commons::Limits;

  enum class ReturnCode
  {
    SUCCESS = 0,
    FAILURE
  };

  class IkError : public std::exception
  {
  public:
    IkError(const std::string &error)
        : _error(error) {}

    virtual const char *what() const noexcept override
    {
      return ("[IK engine]: " + _error).c_str();
    }

  private:
    std::string _error;
  };

} // namespace inverse_kinematics

#endif // INVERSE_KINEMATICS__COMMONS_HPP_
