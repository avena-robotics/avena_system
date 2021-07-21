#include <functional>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>

#include <ompl/config.h>
#include <ompl/base/Goal.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>

#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
//#include <ompl/geometric/planners/experience/LightningRetrieveRepair.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#if OMPL_VERSION_VALUE >= 1001000 // 1.1.0
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#if OMPL_VERSION_VALUE >= 1005000 // 1.5.0
// #include <ompl/geometric/planners/informedtrees/BITstar.h>
#else
#include <ompl/geometric/planners/bitstar/BITstar.h>
#endif
#endif

#include "simPlusPlus/Plugin.h"
#include "config.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

enum StateSpaceType
{
    sim_ompl_statespacetype_position2d,
    sim_ompl_statespacetype_pose2d,
    sim_ompl_statespacetype_position3d,
    sim_ompl_statespacetype_pose3d,
    sim_ompl_statespacetype_joint_position,
    sim_ompl_statespacetype_dubins,
    sim_ompl_statespacetype_cyclic_joint_position
};

enum Algorithm
{

    sim_ompl_algorithm_BiTRRT,
    sim_ompl_algorithm_BITstar,
    sim_ompl_algorithm_BKPIECE1,
    sim_ompl_algorithm_CForest,
    sim_ompl_algorithm_EST,
    sim_ompl_algorithm_FMT,
    sim_ompl_algorithm_KPIECE1,
    sim_ompl_algorithm_LazyPRM,
    sim_ompl_algorithm_LazyPRMstar,
    sim_ompl_algorithm_LazyRRT,
    sim_ompl_algorithm_LBKPIECE1,
    sim_ompl_algorithm_LBTRRT,
    sim_ompl_algorithm_PDST,
    sim_ompl_algorithm_PRM,
    sim_ompl_algorithm_PRMstar,
    sim_ompl_algorithm_pRRT,
    sim_ompl_algorithm_pSBL,
    sim_ompl_algorithm_RRT,
    sim_ompl_algorithm_RRTConnect,
    sim_ompl_algorithm_RRTstar,
    sim_ompl_algorithm_SBL,
    sim_ompl_algorithm_SPARS,
    sim_ompl_algorithm_SPARStwo,
    sim_ompl_algorithm_STRIDE,
    sim_ompl_algorithm_TRRT
};

struct LuaCallbackFunction
{
    // name of the Lua function
    std::string function;
    // id of the script where the function is defined in
    simInt scriptId;
};

struct ObjectDefHeader
{
    // internal handle of this object (used by the plugin):
    simInt handle;
    // name of this object:
    std::string name;
    // objects created during simulation will be destroyed when simulation terminates:
    bool destroyAfterSimulationStop;
};

struct StateSpaceDef
{
    ObjectDefHeader header;
    // type of this state space:
    StateSpaceType type;
    // handle of the object (object, or joint if type = joint_position/cyclic_joint_position):
    simInt objectHandle;
    // object handle in order to specify optional reference frame that is not absolute
    // for sim_ompl_statespace_pose2d, etc.
    simInt refFrameHandle;
    // weight of this state space component (used for state distance calculation):
    simFloat weight;
    // lower bounds of search space:
    std::vector<simFloat> boundsLow;
    // upper bounds of search space:
    std::vector<simFloat> boundsHigh;
    // use this state space as the default projection:
    bool defaultProjection;
    // (specific to dubins state space) turning radius:
    double dubinsTurningRadius;
    // (specific to dubins state space) symmetric:
    bool dubinsIsSymmetric;
};

struct TaskDef
{
    ObjectDefHeader header;
    // state space is a composition of elementary state spaces (internal handles to StateSpaceDef objects):
    std::vector<simInt> stateSpaces;
    // handle of the collision pairs:
    std::vector<simInt> collisionPairHandles;
    // start state:
    std::vector<simFloat> startState;
    // goal can be specified in different ways:
    struct Goal
    {
        enum {STATE, DUMMY_PAIR, CLLBACK} type;
        // goal ref. dummy:
        int refDummy;
        // goal metric:
        float metric[4]; // x,y,z,angle(orientation), relative to refDummy
        // goal tolerance:
        float tolerance;
        // goal state:
        std::vector<std::vector<simFloat> > states;
        // goal dummy pair:
        struct {simInt goalDummy, robotDummy;} dummyPair;
        // goal callback:
        LuaCallbackFunction callback;
    } goal;
    // state validation:
    struct StateValidation
    {
        enum {DEFAULT, CLLBACK} type;
        // state validation callback:
        LuaCallbackFunction callback;
    } stateValidation;
    // resolution at which state validity needs to be verified in order for a
    // motion between two states to be considered valid (specified as a
    // fraction of the space's extent)
    float stateValidityCheckingResolution;
    // state sampling:
    struct ValidStateSampling
    {
        enum {DEFAULT, CLLBACK} type;
        // state sampling callback:
        LuaCallbackFunction callback;
        // "near" state sampling callback:
        LuaCallbackFunction callbackNear;
    } validStateSampling;
    // projection evaluation:
    struct ProjectionEvaluation
    {
        enum {DEFAULT, CLLBACK} type;
        // projection evaluation callback:
        LuaCallbackFunction callback;
        // size of the projection (for callback)
        int dim;
    } projectionEvaluation;
    // search algorithm to use:
    Algorithm algorithm;
    // state space dimension:
    int dim;
    // how many things we should say in the console? (0 = stay silent)
    int verboseLevel;
    // OMPL classes (created with the setup() command):
    // state space
    ob::StateSpacePtr stateSpacePtr;
    // space information
    ob::SpaceInformationPtr spaceInformationPtr;
    // projection evaluator object
    ob::ProjectionEvaluatorPtr projectionEvaluatorPtr;
    // problem definition
    ob::ProblemDefinitionPtr problemDefinitionPtr;
    // planner
    ob::PlannerPtr planner;
};


