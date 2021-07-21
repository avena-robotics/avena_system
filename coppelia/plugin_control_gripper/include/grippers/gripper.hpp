#ifndef PLUGIN_CONTROL_GRIPPER__GRIPPER_HPP
#define PLUGIN_CONTROL_GRIPPER__GRIPPER_HPP

#include "simLib.h"
#include "commons.hpp"

struct InputOptions
{
  std::string prefix;
};

struct OpenGripperOptions
{
  // For now this structure is a placeholder for future values
};

struct CloseGripperOptions
{
  // For now this structure is a placeholder for future values
};

class Gripper
{

public:
  explicit Gripper(const InputOptions &options)
      : _input_options(options){};
  virtual ~Gripper() = default;
  virtual int open(const OpenGripperOptions &options = OpenGripperOptions()) = 0;
  virtual int close(const CloseGripperOptions &options = CloseGripperOptions()) = 0;

  using SharedPtr = std::shared_ptr<Gripper>;

protected:
  InputOptions _input_options;
};

#endif // PLUGIN_CONTROL_GRIPPER__COMMONS_HPP
