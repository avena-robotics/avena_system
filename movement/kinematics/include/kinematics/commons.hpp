#ifndef KINEMATICS__COMMONS_HPP_
#define KINEMATICS__COMMONS_HPP_

// ___CPP___
#include <exception>

// ___ROS___
#include <rclcpp/rclcpp.hpp>

// ___Avena___
#include <helpers_commons/helpers_commons.hpp>

namespace kinematics
{
  // const static rclcpp::Logger LOGGER = rclcpp::get_logger("kinematics");
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
    {
      _error = "[IK engine]: " + error; 
    }

    virtual const char *what() const noexcept override
    {
      return _error.c_str();
    }

  private:
    std::string _error;
  };

} // namespace kinematics

#endif // KINEMATICS__COMMONS_HPP_
