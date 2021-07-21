#ifndef PLUGIN_CONTROL_GRIPPER__FRANKA_GRIPPER_HPP
#define PLUGIN_CONTROL_GRIPPER__FRANKA_GRIPPER_HPP

#include "grippers/gripper.hpp"

class FrankaGripper : public Gripper
{
  using SharedPtr = std::shared_ptr<FrankaGripper>;

public:
  explicit FrankaGripper(const InputOptions &options);
  virtual ~FrankaGripper() = default;
  virtual int open(const OpenGripperOptions &options = OpenGripperOptions()) override;
  virtual int close(const CloseGripperOptions &options = CloseGripperOptions()) override;

private:
  std::string _left_joint_name = "gripper_joint_1";
  std::string _right_joint_name = "gripper_joint_2";
  Handle _left_joint_handle;
  Handle _right_joint_handle;
};

#endif // PLUGIN_CONTROL_GRIPPER__COMMONS_HPP
