#ifndef BULLET_SERVER__SETUP_SCENE_HPP_
#define BULLET_SERVER__SETUP_SCENE_HPP_

// ___CPP___
#include <atomic>

// ___Bullet___
#include <SharedMemory/PhysicsServerExampleBullet2.h>
#include "Bullet3Common/b3CommandLineArgs.h"
#include <CommonInterfaces/CommonExampleInterface.h>
#include <CommonInterfaces/CommonGUIHelperInterface.h>
#include <SharedMemory/SharedMemoryCommon.h>
#include <stdlib.h>

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// ___Avena___
#include <bullet_client/b3RobotSimulatorClientAPI.h>
#include <helpers_commons/helpers_commons.hpp>
#include <helpers_vision/helpers_vision.hpp>

// ___Package___
#include "bullet_server/visibility_control.h"

namespace bullet_server
{
  enum class ReturnCode
  {
    SUCCESS = 0,
    FAILURE
  };

  struct WorkspaceArea
  {
    float x_min;
    float y_min;
    float z_min;

    float x_max;
    float y_max;
    float z_max;
  };

  class SetupScene : public rclcpp::Node, public helpers::WatchdogInterface
  {
  public:
    explicit SetupScene(const rclcpp::NodeOptions &options);
    virtual ~SetupScene();
    virtual void initNode() override;
    virtual void shutDownNode() override;

  private:
    ReturnCode _getParametersFromServer();
    ReturnCode _createWorld();

    helpers::Watchdog::SharedPtr _watchdog;
    WorkspaceArea _workspace_area;
    helpers::commons::RobotInfo _robot_info;
  };

} // namespace bullet_server

#endif // BULLET_SERVER__SETUP_SCENE_HPP_
