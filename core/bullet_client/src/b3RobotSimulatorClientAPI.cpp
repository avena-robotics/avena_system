#include "bullet_client/b3RobotSimulatorClientAPI.h"

#include "SharedMemory/PhysicsClientC_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#ifdef BT_ENABLE_ENET
#include "SharedMemory/PhysicsClientUDP_C_API.h"
#endif //PHYSICS_UDP

#ifdef BT_ENABLE_CLSOCKET
#include "SharedMemory/PhysicsClientTCP_C_API.h"
#endif //PHYSICS_TCP

#include "SharedMemory/PhysicsDirectC_API.h"

#include "SharedMemory/SharedMemoryInProcessPhysicsC_API.h"

#include "SharedMemory/SharedMemoryPublic.h"
#include "Bullet3Common/b3Logging.h"

#ifdef BT_ENABLE_GRPC
#include "SharedMemory/PhysicsClientGRPC_C_API.h"
#endif

namespace bullet_client
{

	b3RobotSimulatorClientAPI::b3RobotSimulatorClientAPI()
	{
	}

	b3RobotSimulatorClientAPI::~b3RobotSimulatorClientAPI()
	{
	}

	void b3RobotSimulatorClientAPI::removeAllUserDebugItems()
	{
		b3SharedMemoryCommandHandle commandHandle;
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;
		commandHandle = b3InitUserDebugDrawRemoveAll(m_data->m_physicsClientHandle);
		statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
		statusType = b3GetStatusType(statusHandle);
	}

	bool b3RobotSimulatorClientAPI::calculateIK(const struct b3RobotSimulatorInverseKinematicArgs &args, struct b3RobotSimulatorInverseKinematicsResults &results)
	{
		if (!isConnected())
		{
			b3Warning("Not connected");
			return false;
		}
		btAssert(args.m_endEffectorLinkIndex >= 0);
		btAssert(args.m_bodyUniqueId >= 0);

		b3SharedMemoryCommandHandle command = b3CalculateInverseKinematicsCommandInit(m_data->m_physicsClientHandle, args.m_bodyUniqueId);
		if ((args.m_flags & B3_HAS_IK_TARGET_ORIENTATION) && (args.m_flags & B3_HAS_NULL_SPACE_VELOCITY))
		{
			b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(command, args.m_numDegreeOfFreedom, args.m_endEffectorLinkIndex, args.m_endEffectorTargetPosition, args.m_endEffectorTargetOrientation, &args.m_lowerLimits[0], &args.m_upperLimits[0], &args.m_jointRanges[0], &args.m_restPoses[0]);
		}
		else if (args.m_flags & B3_HAS_IK_TARGET_ORIENTATION)
		{
			b3CalculateInverseKinematicsAddTargetPositionWithOrientation(command, args.m_endEffectorLinkIndex, args.m_endEffectorTargetPosition, args.m_endEffectorTargetOrientation);
		}
		else if (args.m_flags & B3_HAS_NULL_SPACE_VELOCITY)
		{
			b3CalculateInverseKinematicsPosWithNullSpaceVel(command, args.m_numDegreeOfFreedom, args.m_endEffectorLinkIndex, args.m_endEffectorTargetPosition, &args.m_lowerLimits[0], &args.m_upperLimits[0], &args.m_jointRanges[0], &args.m_restPoses[0]);
		}
		else
		{
			b3CalculateInverseKinematicsAddTargetPurePosition(command, args.m_endEffectorLinkIndex, args.m_endEffectorTargetPosition);
		}

		if (args.m_flags & B3_HAS_JOINT_DAMPING)
		{
			b3CalculateInverseKinematicsSetJointDamping(command, args.m_numDegreeOfFreedom, &args.m_jointDamping[0]);
		}

		if (args.m_flags & B3_HAS_CURRENT_POSITIONS)
		{
			b3CalculateInverseKinematicsSetCurrentPositions(command, args.m_numDegreeOfFreedom, &args.m_currentJointPositions[0]);
		}

		b3CalculateInverseKinematicsSetResidualThreshold(command, 1e-4);
		b3CalculateInverseKinematicsSetMaxNumIterations(command, 1000);
		b3CalculateInverseKinematicsSelectSolver(command, IK_DLS);

		b3SharedMemoryStatusHandle statusHandle;
		statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);

		int numPos = 0;

		bool result = b3GetStatusInverseKinematicsJointPositions(statusHandle,
																 &results.m_bodyUniqueId,
																 &numPos,
																 0) != 0;
		if (result && numPos)
		{
			results.m_calculatedJointPositions.resize(numPos);
			result = b3GetStatusInverseKinematicsJointPositions(statusHandle,
																&results.m_bodyUniqueId,
																&numPos,
																&results.m_calculatedJointPositions[0]) != 0;
		}
		return result;
	}

	void b3RobotSimulatorClientAPI::performCollisionDetection()
	{
		if (!isConnected())
		{
			b3Warning("Not connected");
			return;
		}

		b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;

		if (b3CanSubmitCommand(sm))
		{
			statusHandle = b3SubmitClientCommandAndWaitStatus(sm, b3InitPerformCollisionDetectionCommand(sm));
		}
	}

	btQuaternion b3RobotSimulatorClientAPI::getDifferenceQuaternion(const btQuaternion &quaternionStart, const btQuaternion &quaternionEnd)
	{
		double quatStart[] = {quaternionStart.getX(), quaternionStart.getY(), quaternionStart.getZ(), quaternionStart.getW()};
		double quatEnd[] = {quaternionEnd.getX(), quaternionEnd.getY(), quaternionEnd.getZ(), quaternionEnd.getW()};
		int physicsClientId = 0;
		int hasQuatStart = 0;
		int hasQuatEnd = 0;

		double quatOut[4];
		b3GetQuaternionDifference(quatStart, quatEnd, quatOut);

		if (!quatOut)
			return btQuaternion();

		btQuaternion quat_diff;
		quat_diff[0] = quatOut[0];
		quat_diff[1] = quatOut[1];
		quat_diff[2] = quatOut[2];
		quat_diff[3] = quatOut[3];
		return quat_diff;
	}

	void b3RobotSimulatorClientAPI::renderScene()
	{
		if (!isConnected())
		{
			b3Warning("Not connected");
			return;
		}
		if (m_data->m_guiHelper)
		{
			b3InProcessRenderSceneInternal(m_data->m_physicsClientHandle);
		}
	}

	void b3RobotSimulatorClientAPI::debugDraw(int debugDrawMode)
	{
		if (!isConnected())
		{
			b3Warning("Not connected");
			return;
		}
		if (m_data->m_guiHelper)
		{
			b3InProcessDebugDrawInternal(m_data->m_physicsClientHandle, debugDrawMode);
		}
	}

	bool b3RobotSimulatorClientAPI::mouseMoveCallback(float x, float y)
	{
		if (!isConnected())
		{
			b3Warning("Not connected");
			return false;
		}
		if (m_data->m_guiHelper)
		{
			return b3InProcessMouseMoveCallback(m_data->m_physicsClientHandle, x, y) != 0;
		}
		return false;
	}
	bool b3RobotSimulatorClientAPI::mouseButtonCallback(int button, int state, float x, float y)
	{
		if (!isConnected())
		{
			b3Warning("Not connected");
			return false;
		}
		if (m_data->m_guiHelper)
		{
			return b3InProcessMouseButtonCallback(m_data->m_physicsClientHandle, button, state, x, y) != 0;
		}
		return false;
	}

	bool b3RobotSimulatorClientAPI::connect(int mode, const std::string &hostName, int portOrKey)
	{
		if (m_data->m_physicsClientHandle)
		{
			b3Warning("Already connected, disconnect first.");
			return false;
		}
		b3PhysicsClientHandle sm = 0;

		int udpPort = 1234;
		int tcpPort = 6667;
		int key = SHARED_MEMORY_KEY;

		switch (mode)
		{
		case eCONNECT_EXISTING_EXAMPLE_BROWSER:
		{
			sm = b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect(m_data->m_guiHelper);
			break;
		}

		case eCONNECT_GUI:
		{
			int argc = 0;
			char *argv[1] = {0};
#ifdef __APPLE__
			sm = b3CreateInProcessPhysicsServerAndConnectMainThread(argc, argv);
#else
			sm = b3CreateInProcessPhysicsServerAndConnect(argc, argv);
#endif
			break;
		}
		case eCONNECT_GUI_SERVER:
		{
			int argc = 0;
			char *argv[1] = {0};
#ifdef __APPLE__
			sm = b3CreateInProcessPhysicsServerAndConnectMainThread(argc, argv);
#else
			sm = b3CreateInProcessPhysicsServerAndConnect(argc, argv);
#endif
			break;
		}
		case eCONNECT_DIRECT:
		{
			sm = b3ConnectPhysicsDirect();
			break;
		}
		case eCONNECT_SHARED_MEMORY:
		{
			if (portOrKey >= 0)
			{
				key = portOrKey;
			}
			sm = b3ConnectSharedMemory(key);
			break;
		}
		case eCONNECT_UDP:
		{
			if (portOrKey >= 0)
			{
				udpPort = portOrKey;
			}
#ifdef BT_ENABLE_ENET

			sm = b3ConnectPhysicsUDP(hostName.c_str(), udpPort);
#else
			b3Warning("UDP is not enabled in this build");
#endif //BT_ENABLE_ENET

			break;
		}
		case eCONNECT_TCP:
		{
			if (portOrKey >= 0)
			{
				tcpPort = portOrKey;
			}
#ifdef BT_ENABLE_CLSOCKET

			sm = b3ConnectPhysicsTCP(hostName.c_str(), tcpPort);
#else
			b3Warning("TCP is not enabled in this pybullet build");
#endif //BT_ENABLE_CLSOCKET
			break;
		}
		case eCONNECT_GRPC:
		{
#ifdef BT_ENABLE_GRPC
			sm = b3ConnectPhysicsGRPC(hostName.c_str(), tcpPort);
#else
			b3Warning("GRPC is not enabled in this pybullet build");
#endif
			break;
		}
		default:
		{
			b3Warning("connectPhysicsServer unexpected argument");
		}
		};

		if (sm)
		{
			m_data->m_physicsClientHandle = sm;
			if (!b3CanSubmitCommand(m_data->m_physicsClientHandle))
			{
				disconnect();
				return false;
			}
			return true;
		}
		return false;
	}
} // namespace bullet_client
