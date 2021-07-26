#ifndef BULLET_SERVER__BULLET_SERVER_HPP_
#define BULLET_SERVER__BULLET_SERVER_HPP_

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

// ___Avena___
#include <helpers_commons/helpers_commons.hpp>

// ___Package___
#include "bullet_server/visibility_control.h"

// This is some global and static variables which are used somewhere else so better leave them be
int gSharedMemoryKey = -1;

static SharedMemoryCommon *example = NULL;
static bool interrupted = false;

#ifndef _WIN32
#include <signal.h>
#include <err.h>
#include <unistd.h>
static void cleanup(int signo)
{
  std::cout << "Calling cleanup. Interrupt flag is " << std::boolalpha << interrupted << std::noboolalpha << std::endl;
  if (interrupted)
  { // this is the second time, we're hanging somewhere
    //  if (example) {
    //      example->abort();
    //  }
    b3Printf("Aborting and deleting SharedMemoryCommon object");
    sleep(1);
    delete example;
    errx(EXIT_FAILURE, "aborted example on signal %d", signo);
  }
  interrupted = true;
  warnx("caught signal %d", signo);
}
#endif //_WIN32

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

  class BulletServer : public rclcpp::Node
  {
  public:
    explicit BulletServer(const rclcpp::NodeOptions &options);
    virtual ~BulletServer();

  private:
    std::thread _server_thread;
  };

} // namespace bullet_server

#endif // BULLET_SERVER__BULLET_SERVER_HPP_
