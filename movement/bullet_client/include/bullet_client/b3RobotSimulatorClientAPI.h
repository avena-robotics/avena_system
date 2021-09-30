#ifndef BULLET_CLIENT__B3_ROBOT_SIMULATOR_CLIENT_API_GUI_H
#define BULLET_CLIENT__B3_ROBOT_SIMULATOR_CLIENT_API_GUI_H

#include <SharedMemory/b3RobotSimulatorClientAPI_NoGUI.h>
#include <memory>
#include <iostream>
#include <vector>

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

    int createCollisionShapeBox(const btVector3 &position, const btQuaternion &orientation, const btVector3 &dims);
    
    int createVisualShapeBox(const btVector3 &position, const btQuaternion &orientation, const btVector3 &dims, const btVector4 &rgba = btVector4(1, 1, 1, 1));

    /**
     * @brief Create a Collision Shape Mesh object
     * 
     * @param vertices flat sequence of point cloud coordinates: pt_0_x, pt_0_y, pt_0_z, pt_1_x, pt_1_y, pt_1_z, pt_2_x, pt_2_y, pt_2_z, ...
     * (check physics_client_handler::createOctomap for example how to convert pcl::PointCloud to vertices)
     * @param indices of points in point cloud belonging to specific triangular polygon
     * @return int unique ID of spawned collision mesh
     */
    int createCollisionShapeMesh(const std::vector<double> &vertices, const std::vector<int> &indices);
    
    
    int createVisualShapeMesh(const std::vector<double> &vertices, const std::vector<int> &indices, const btVector4 &rgba = btVector4(1, 1, 1, 1));

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
