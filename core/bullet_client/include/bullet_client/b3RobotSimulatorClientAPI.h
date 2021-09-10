#ifndef BULLET_CLIENT__B3_ROBOT_SIMULATOR_CLIENT_API_GUI_H
#define BULLET_CLIENT__B3_ROBOT_SIMULATOR_CLIENT_API_GUI_H

#include <SharedMemory/b3RobotSimulatorClientAPI_NoGUI.h>
#include <memory>
#include <iostream>

namespace bullet_client
{
  ///The b3RobotSimulatorClientAPI_GUI is pretty much the C++ version of pybullet
  ///as documented in the pybullet Quickstart Guide
  ///https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA
  class b3RobotSimulatorClientAPI : public b3RobotSimulatorClientAPI_NoGUI
  {
  public:
    using SharedPtr = std::shared_ptr<b3RobotSimulatorClientAPI>;

    b3RobotSimulatorClientAPI();

    virtual ~b3RobotSimulatorClientAPI();

  	virtual void removeAllUserDebugItems();

    bool calculateIK(const struct b3RobotSimulatorInverseKinematicArgs& args, struct b3RobotSimulatorInverseKinematicsResults& results);

    btQuaternion getDifferenceQuaternion(const btQuaternion &quaternionStart, const btQuaternion &quaternionEnd);

    /**
     * @brief It looks like it does not work
     * 
     */
    void performCollisionDetection();

    virtual bool connect(int mode, const std::string &hostName = "localhost", int portOrKey = -1);

    virtual void renderScene();

    virtual void debugDraw(int debugDrawMode);

    virtual bool mouseMoveCallback(float x, float y);

    virtual bool mouseButtonCallback(int button, int state, float x, float y);
  };

} // namespace bullet_client

#endif //B3_ROBOT_SIMULATOR_CLIENT_API_H
