#ifndef HELPERS_COMMONS__ROBOT_UTILS_HPP_
#define HELPERS_COMMONS__ROBOT_UTILS_HPP_

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <urdf_parser/urdf_parser.h>

// ___Helpers___
#include "helpers_commons/structures.hpp"

namespace helpers
{
  namespace commons
  {
    /**
     * @brief Get the Robot Description object
     * 
     * @param node 
     * @return std::string 
     */
    std::string getRobotDescription();

    /**
     * @brief Get the Robot Links Names object
     * 
     * @param node 
     * @return std::vector<std::string> 
     */
    std::vector<std::string> getRobotLinksNames();

    /**
     * @brief Get the Robot Info with basic information about robot e.g. prefix name, list of joints, list of links
     * 
     * @param side working side Right now it does not really matter what you put here. All is read from URDF. 
     * Kept for now for backward compatibility.
     * @return RobotInfo 
     */
    RobotInfo getRobotInfo(const std::string &side = "none");
  } // namespace commons
} // namespace helpers

#endif // HELPERS_COMMONS__ROBOT_UTILS_HPP_
