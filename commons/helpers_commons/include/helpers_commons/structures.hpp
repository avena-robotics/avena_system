#ifndef HELPERS_COMMONS__STRUCTURES_HPP_
#define HELPERS_COMMONS__STRUCTURES_HPP_

// ___CPP___
#include <vector>
#include <string>

namespace helpers
{
  struct Frames
  {
    inline const static std::string world_frame = "world";
    inline const static std::string panda_wrist_frame = "panda_link8";
    inline const static std::string camera_frame = "head_camera1_rgb_optical_frame";
    inline const static std::string camera_frame2 = "head_camera2_rgb_optical_frame";
  };

  struct CameraIntrinsic
  {
    size_t width;
    size_t height;
    float cx;
    float cy;
    float fx;
    float fy;
  };

  namespace commons
  {
    /**
     * @brief Specific information about currently used gripper, but there is nothing in it right now...
     */
    struct GripperInfo
    {
      std::vector<std::string> link_names;
    };

    /**
     * @brief Joint limits
     */
    struct Limits
    {
      Limits() = default;
      Limits(const double &lower, const double &upper)
          : lower(lower), upper(upper) {}
      double lower;
      double upper;
    };

    /**
     * @brief Contains basic information about robotic arm with list of links, joints and gripper information
     */
    struct RobotInfo
    {
      std::string robot_name;
      std::string robot_prefix;
      std::string connection;
      std::vector<std::string> joint_names;
      std::vector<std::string> fixed_joint_names;
      std::vector<std::string> link_names;
      size_t nr_joints;
      size_t nr_fixed_joints;
      size_t nr_links;
      std::vector<Limits> limits;
      std::vector<Limits> soft_limits;

      GripperInfo gripper_info;
    };
  } // namespace commons
} // namespace helpers

#endif // HELPERS_COMMONS__STRUCTURES_HPP_
