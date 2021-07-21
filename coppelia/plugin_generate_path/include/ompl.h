#include "plugin.h"

// temporary solution global values
int g_selected_item_id;
int g_selected_item_handle;
int g_dummyWorld_handle;
int g_item_to_detach_handle;
float g_validate_min_distance;

std::vector<std::string> g_joint_names;
std::vector<Bounds> g_bounds;


float g_constraint_tolerance;
Eigen::Matrix3f g_world_rotation;
Eigen::Matrix3f g_item_rotation;
Eigen::Matrix3f g_item_transform;
std::vector<float> g_item_matrix(16);

std::vector<float> g_goal_place(7);
int g_goal_place_id;

Eigen::Matrix3f getRotation(std::vector<float> matrix) //!ok
{
    Eigen::Matrix3f rotation;
    rotation << matrix[0], matrix[1], matrix[2], matrix[4], matrix[5], matrix[6], matrix[8], matrix[8], matrix[10];

    return rotation;
}

namespace ob = ompl::base;
namespace og = ompl::geometric;

#if OMPL_VERSION_VALUE >= 1004000 // 1.4.0
typedef Eigen::Ref<Eigen::VectorXd> OMPLProjection;
#else // All other versions
typedef ompl::base::EuclideanProjection &OMPLProjection;
#endif

class ProjectionEvaluator : public ob::ProjectionEvaluator
{
public:
    ProjectionEvaluator(std::map<simInt, StateSpaceDef *> &ss, const ob::StateSpacePtr &space, TaskDef *task)
        : ob::ProjectionEvaluator(space), statespaces(ss), statespace(space)
    {
        this->task = task;

        dim = 0;

        switch (task->projectionEvaluation.type)
        {
        case TaskDef::ProjectionEvaluation::DEFAULT:
            switch (task->goal.type)
            {
            case TaskDef::Goal::STATE:
            case TaskDef::Goal::CLLBACK:
                dim = defaultProjectionSize();
                break;
            case TaskDef::Goal::DUMMY_PAIR:
                dim = dummyPairProjectionSize();
                break;
            default:
                // this will never happen
                dim = 0;
                break;
            }
            break;
        case TaskDef::ProjectionEvaluation::CLLBACK:
            dim = luaProjectCallbackSize();
            break;
        default:
            // this will never happen
            dim = 0;
            break;
        }
    }

    virtual unsigned int getDimension(void) const
    {
        return dim;
    }

    virtual void defaultCellSizes(void)
    {
        // TODO: handle this in the plugin API
        cellSizes_.resize(dim);
        for (int i = 0; i < dim; i++)
            cellSizes_[i] = 0.05;
    }

    virtual void project(const ob::State *state, OMPLProjection projection) const
    {
        for (int i = 0; i < dim; i++)
            projection(i) = 0.0;

        // const ob::CompoundState *s = state->as<ob::CompoundStateSpace::StateType>();

        switch (task->projectionEvaluation.type)
        {
        case TaskDef::ProjectionEvaluation::DEFAULT:
            switch (task->goal.type)
            {
            case TaskDef::Goal::STATE:
            case TaskDef::Goal::CLLBACK:
                defaultProjection(state, projection);
                break;
            case TaskDef::Goal::DUMMY_PAIR:
                dummyPairProjection(state, projection);
                break;
            }
            break;
        case TaskDef::ProjectionEvaluation::CLLBACK:
            luaProjectCallback(state, projection);
            break;
        }
    }

protected:
    virtual int defaultProjectionSize() const
    {
        for (size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];

            if (!stateSpace->defaultProjection)
                continue;

            switch (stateSpace->type)
            {
            case sim_ompl_statespacetype_pose2d:
            case sim_ompl_statespacetype_position2d:
                return 2;
            case sim_ompl_statespacetype_pose3d:
            case sim_ompl_statespacetype_position3d:
                return 3;
            case sim_ompl_statespacetype_joint_position:
            case sim_ompl_statespacetype_cyclic_joint_position:
                return 1;
            case sim_ompl_statespacetype_dubins:
                return 2;
            }
        }

        return 0;
    }

    virtual void defaultProjection(const ob::State *state, OMPLProjection projection) const
    {
        const ob::CompoundState *s = state->as<ob::CompoundStateSpace::StateType>();

        for (size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];

            if (!stateSpace->defaultProjection)
                continue;

            switch (stateSpace->type)
            {
            case sim_ompl_statespacetype_pose2d:
                projection(0) = s->as<ob::SE2StateSpace::StateType>(i)->getX();
                projection(1) = s->as<ob::SE2StateSpace::StateType>(i)->getY();
                break;
            case sim_ompl_statespacetype_pose3d:
                projection(0) = s->as<ob::SE3StateSpace::StateType>(i)->getX();
                projection(1) = s->as<ob::SE3StateSpace::StateType>(i)->getY();
                projection(2) = s->as<ob::SE3StateSpace::StateType>(i)->getZ();
                break;
            case sim_ompl_statespacetype_position2d:
                projection(0) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                projection(1) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[1];
                break;
            case sim_ompl_statespacetype_position3d:
                projection(0) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                projection(1) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[1];
                projection(2) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[2];
                break;
            case sim_ompl_statespacetype_joint_position:
                projection(0) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                break;
            case sim_ompl_statespacetype_cyclic_joint_position:
                projection(0) = s->as<ob::SO2StateSpace::StateType>(i)->value;
                break;
            case sim_ompl_statespacetype_dubins:
                projection(0) = s->as<ob::SE2StateSpace::StateType>(i)->getX();
                projection(1) = s->as<ob::SE2StateSpace::StateType>(i)->getY();
                break;
            }

            break;
        }
    }

    virtual int dummyPairProjectionSize() const
    {
        /*
        return 3;
        */
        int s = 0;
        for (int i = 0; i < 3; i++)
        {
            if (task->goal.metric[i] != 0.0)
                s++;
        }
        if (s == 0)
            s = 1; // if X/Y/Z are ignored
        return s;
    }

    virtual void dummyPairProjection(const ob::State *state, OMPLProjection projection) const
    {
        /*
        simFloat pos[3];
        simGetObjectPosition(task->goal.dummyPair.robotDummy, -1, &pos[0]);
        projection(0) = pos[0];
        projection(1) = pos[1];
        projection(2) = pos[2];
        */

        // TODO: don't we need to apply the provided state to the robot, read the tip dummy's position, then project it?

        // do projection, only for axis that should not be ignored:
        simFloat pos[3];
        simGetObjectPosition(task->goal.dummyPair.robotDummy, task->goal.refDummy, &pos[0]);
        int ind = 0;
        for (int i = 0; i < 3; i++)
        {
            if (task->goal.metric[i] != 0.0)
                projection(ind++) = pos[i];
        }
        if (ind == 0)
            projection(0) = 0.0; // if X/Y/Z are ignored

        // TODO: restore original state, no?
    }

    virtual int luaProjectCallbackSize() const
    {
        return task->projectionEvaluation.dim;
    }

    virtual void luaProjectCallback(const ob::State *state, OMPLProjection projection) const
    {
        // std::vector<double> stateVec;
        // statespace->copyToReals(stateVec, state);

        // const ob::CompoundState *s = state->as<ob::CompoundStateSpace::StateType>();

        // projectionEvaluationCallback_in in_args;
        // projectionEvaluationCallback_out out_args;

        // for(size_t i = 0; i < stateVec.size(); i++)
        //     in_args.state.push_back((float)stateVec[i]);

        // if(projectionEvaluationCallback(task->projectionEvaluation.callback.scriptId,
        // task->projectionEvaluation.callback.function.c_str(), &in_args, &out_args))
        // {
        //     for(size_t i = 0; i < out_args.projection.size(); i++)
        //     {
        //         projection(i) = out_args.projection[i];
        //     }
        // }
        // else
        // {
        //     throw ompl::Exception("Projection evaluation callback " + task->projectionEvaluation.callback.function +
        //     " returned an error");
        // }
    }

    std::map<simInt, StateSpaceDef *> &statespaces;
    TaskDef *task;
    const ob::StateSpacePtr &statespace;
    int dim;
};

class StateSpace : public ob::CompoundStateSpace
{
public:
    StateSpace(std::map<simInt, StateSpaceDef *> &ss, TaskDef *task)
        : ob::CompoundStateSpace(), statespaces(ss), task(task)
    {
        setName("SimCompoundStateSpace");
        type_ = ompl::base::STATE_SPACE_TYPE_COUNT + 1;

        for (size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];

            ob::StateSpacePtr subSpace;

            switch (stateSpace->type)
            {
            case sim_ompl_statespacetype_pose2d:
                subSpace = ob::StateSpacePtr(new ob::SE2StateSpace());
                subSpace->as<ob::CompoundStateSpace>()->getSubspace(0)->setName(stateSpace->header.name + ".positio"
                                                                                                          "n");
                subSpace->as<ob::CompoundStateSpace>()->getSubspace(1)->setName(stateSpace->header.name + ".orienta"
                                                                                                          "tion");
                break;
            case sim_ompl_statespacetype_pose3d:
                subSpace = ob::StateSpacePtr(new ob::SE3StateSpace());
                subSpace->as<ob::CompoundStateSpace>()->getSubspace(0)->setName(stateSpace->header.name + ".positio"
                                                                                                          "n");
                subSpace->as<ob::CompoundStateSpace>()->getSubspace(1)->setName(stateSpace->header.name + ".orienta"
                                                                                                          "tion");
                break;
            case sim_ompl_statespacetype_position2d:
                subSpace = ob::StateSpacePtr(new ob::RealVectorStateSpace(2));
                break;
            case sim_ompl_statespacetype_position3d:
                subSpace = ob::StateSpacePtr(new ob::RealVectorStateSpace(3));
                break;
            case sim_ompl_statespacetype_joint_position:
                subSpace = ob::StateSpacePtr(new ob::RealVectorStateSpace(1));
                break;
            case sim_ompl_statespacetype_cyclic_joint_position:
                subSpace = ob::StateSpacePtr(new ob::SO2StateSpace());
                break;
            case sim_ompl_statespacetype_dubins:
                subSpace = ob::StateSpacePtr(
                    new ob::DubinsStateSpace(stateSpace->dubinsTurningRadius, stateSpace->dubinsIsSymmetric));
                subSpace->as<ob::CompoundStateSpace>()->getSubspace(0)->setName(stateSpace->header.name + ".positio"
                                                                                                          "n");
                subSpace->as<ob::CompoundStateSpace>()->getSubspace(1)->setName(stateSpace->header.name + ".orienta"
                                                                                                          "tion");
                break;
            }

            subSpace->setName(stateSpace->header.name);
            addSubspace(subSpace, stateSpace->weight);

            // set bounds:

            ob::RealVectorBounds bounds(stateSpace->boundsLow.size());
            ;
            for (size_t j = 0; j < stateSpace->boundsLow.size(); j++)
                bounds.setLow(j, stateSpace->boundsLow[j]);
            for (size_t j = 0; j < stateSpace->boundsHigh.size(); j++)
                bounds.setHigh(j, stateSpace->boundsHigh[j]);

            switch (stateSpace->type)
            {
            case sim_ompl_statespacetype_pose2d:
                as<ob::SE2StateSpace>(i)->setBounds(bounds);
                break;
            case sim_ompl_statespacetype_pose3d:
                as<ob::SE3StateSpace>(i)->setBounds(bounds);
                break;
            case sim_ompl_statespacetype_position2d:
                as<ob::RealVectorStateSpace>(i)->setBounds(bounds);
                break;
            case sim_ompl_statespacetype_position3d:
                as<ob::RealVectorStateSpace>(i)->setBounds(bounds);
                break;
            case sim_ompl_statespacetype_joint_position:
                as<ob::RealVectorStateSpace>(i)->setBounds(bounds);
                break;
            case sim_ompl_statespacetype_cyclic_joint_position:
                if (!stateSpace->boundsLow.empty() || !stateSpace->boundsHigh.empty())
                    // sim::addLog(sim_verbosity_warnings, "cyclic_joint_position state space has no bounds");
                    break;
            case sim_ompl_statespacetype_dubins:
                as<ob::SE2StateSpace>(i)->setBounds(bounds);
                break;
            }
        }
    }

    // writes state s to CoppeliaSim:
    void writeState(const ob::ScopedState<ob::CompoundStateSpace> &s)
    {
        // int j = 0;
        simFloat pos[3], orient[4], value;

        for (size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];

            switch (stateSpace->type)
            {
            case sim_ompl_statespacetype_pose2d:
                simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                simGetObjectOrientation(stateSpace->objectHandle, stateSpace->refFrameHandle,
                                        &orient[0]); // Euler angles
                pos[0] = (float)s->as<ob::SE2StateSpace::StateType>(i)->getX();
                pos[1] = (float)s->as<ob::SE2StateSpace::StateType>(i)->getY();
                orient[2] = (float)s->as<ob::SE2StateSpace::StateType>(i)->getYaw();
                simSetObjectOrientation(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]);
                simSetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                break;
            case sim_ompl_statespacetype_pose3d:
                pos[0] = (float)s->as<ob::SE3StateSpace::StateType>(i)->getX();
                pos[1] = (float)s->as<ob::SE3StateSpace::StateType>(i)->getY();
                pos[2] = (float)s->as<ob::SE3StateSpace::StateType>(i)->getZ();
                orient[0] = (float)s->as<ob::SE3StateSpace::StateType>(i)->rotation().x;
                orient[1] = (float)s->as<ob::SE3StateSpace::StateType>(i)->rotation().y;
                orient[2] = (float)s->as<ob::SE3StateSpace::StateType>(i)->rotation().z;
                orient[3] = (float)s->as<ob::SE3StateSpace::StateType>(i)->rotation().w;
                simSetObjectQuaternion(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]);
                simSetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                break;
            case sim_ompl_statespacetype_position2d:
                simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                pos[0] = (float)s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                pos[1] = (float)s->as<ob::RealVectorStateSpace::StateType>(i)->values[1];
                simSetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                break;
            case sim_ompl_statespacetype_position3d:
                pos[0] = (float)s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                pos[1] = (float)s->as<ob::RealVectorStateSpace::StateType>(i)->values[1];
                pos[2] = (float)s->as<ob::RealVectorStateSpace::StateType>(i)->values[2];
                simSetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                break;
            case sim_ompl_statespacetype_joint_position:
                value = (float)s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                simSetJointPosition(stateSpace->objectHandle, value);
                break;
            case sim_ompl_statespacetype_cyclic_joint_position:
                value = (float)s->as<ob::SO2StateSpace::StateType>(i)->value;
                simSetJointPosition(stateSpace->objectHandle, value);
                break;
            case sim_ompl_statespacetype_dubins:
                simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                simGetObjectOrientation(stateSpace->objectHandle, stateSpace->refFrameHandle,
                                        &orient[0]); // Euler angles
                pos[0] = (float)s->as<ob::SE2StateSpace::StateType>(i)->getX();
                pos[1] = (float)s->as<ob::SE2StateSpace::StateType>(i)->getY();
                orient[2] = (float)s->as<ob::SE2StateSpace::StateType>(i)->getYaw();
                simSetObjectOrientation(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]);
                simSetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                break;
            }
        }
    }

    // reads state s from CoppeliaSim:
    void readState(ob::ScopedState<ob::CompoundStateSpace> &s)
    {
        simFloat pos[3], orient[4], value;

        for (size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];

            switch (stateSpace->type)
            {
            case sim_ompl_statespacetype_pose2d:
                simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                simGetObjectOrientation(stateSpace->objectHandle, stateSpace->refFrameHandle,
                                        &orient[0]); // Euler angles
                s->as<ob::SE2StateSpace::StateType>(i)->setXY(pos[0], pos[1]);
                s->as<ob::SE2StateSpace::StateType>(i)->setYaw(orient[2]);
                break;
            case sim_ompl_statespacetype_pose3d:
                simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                simGetObjectQuaternion(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]);
                s->as<ob::SE3StateSpace::StateType>(i)->setXYZ(pos[0], pos[1], pos[2]);
                s->as<ob::SE3StateSpace::StateType>(i)->rotation().x = orient[0];
                s->as<ob::SE3StateSpace::StateType>(i)->rotation().y = orient[1];
                s->as<ob::SE3StateSpace::StateType>(i)->rotation().z = orient[2];
                s->as<ob::SE3StateSpace::StateType>(i)->rotation().w = orient[3];
                break;
            case sim_ompl_statespacetype_position2d:
                simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                s->as<ob::RealVectorStateSpace::StateType>(i)->values[0] = pos[0];
                s->as<ob::RealVectorStateSpace::StateType>(i)->values[1] = pos[1];
                break;
            case sim_ompl_statespacetype_position3d:
                simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                s->as<ob::RealVectorStateSpace::StateType>(i)->values[0] = pos[0];
                s->as<ob::RealVectorStateSpace::StateType>(i)->values[1] = pos[1];
                s->as<ob::RealVectorStateSpace::StateType>(i)->values[2] = pos[2];
                break;
            case sim_ompl_statespacetype_joint_position:
                simGetJointPosition(stateSpace->objectHandle, &value);
                s->as<ob::RealVectorStateSpace::StateType>(i)->values[0] = value;
                break;
            case sim_ompl_statespacetype_cyclic_joint_position:
                simGetJointPosition(stateSpace->objectHandle, &value);
                s->as<ob::SO2StateSpace::StateType>(i)->value = value;
                break;
            case sim_ompl_statespacetype_dubins:
                simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                simGetObjectOrientation(stateSpace->objectHandle, stateSpace->refFrameHandle,
                                        &orient[0]); // Euler angles
                s->as<ob::SE2StateSpace::StateType>(i)->setXY(pos[0], pos[1]);
                s->as<ob::SE2StateSpace::StateType>(i)->setYaw(orient[2]);
                break;
            }
        }
    }

    // Store relative pose of objects:
    void saveRelPoseState(std::vector<float> &p)
    {
        for (size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];
            if (stateSpace->type == sim_ompl_statespacetype_pose2d ||
                stateSpace->type == sim_ompl_statespacetype_pose3d ||
                stateSpace->type == sim_ompl_statespacetype_position2d ||
                stateSpace->type == sim_ompl_statespacetype_position3d)
            {
                p.resize(p.size() + 7);
                simGetObjectPosition(stateSpace->objectHandle, sim_handle_parent, &p[p.size() - 7]);
                simGetObjectQuaternion(stateSpace->objectHandle, sim_handle_parent, &p[p.size() - 4]);
            }
        }
    }

    // Restore relative pose of objects:
    void restoreRelPoseState(const std::vector<float> &p)
    {
        int pt = 0;
        for (size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];
            if (stateSpace->type == sim_ompl_statespacetype_pose2d ||
                stateSpace->type == sim_ompl_statespacetype_pose3d ||
                stateSpace->type == sim_ompl_statespacetype_position2d ||
                stateSpace->type == sim_ompl_statespacetype_position3d)
            {
                simSetObjectPosition(stateSpace->objectHandle, sim_handle_parent, &p[pt + 0]);
                simSetObjectQuaternion(stateSpace->objectHandle, sim_handle_parent, &p[pt + 3]);
                pt += 7;
            }
        }
    }

protected:
    std::map<simInt, StateSpaceDef *> &statespaces;
    TaskDef *task;
};

class StateValidityChecker : public ob::StateValidityChecker
{
public:
    StateValidityChecker(const ob::SpaceInformationPtr &si, TaskDef *task)
        : ob::StateValidityChecker(si), statespace(si->getStateSpace()), task(task)
    {
    }

    virtual ~StateValidityChecker()
    {
    }

    virtual bool isValid(const ob::State *state) const
    {
        switch (task->stateValidation.type)
        {
        case TaskDef::StateValidation::DEFAULT:
            return checkDefault(state);
        case TaskDef::StateValidation::CLLBACK:
            return checkCallback(state);
        }
        return false;
    }

protected:
    virtual bool checkDefault(const ob::State *state) const
    {
        // ob::CompoundStateSpace *ss = statespace->as<ob::CompoundStateSpace>();
        ob::ScopedState<ob::CompoundStateSpace> s(statespace);
        s = state;

        // save old state:
        ob::ScopedState<ob::CompoundStateSpace> s_old(statespace);
        statespace->as<StateSpace>()->readState(s_old);
        std::vector<float> pose_old;
        statespace->as<StateSpace>()->saveRelPoseState(pose_old);

        // write query state:
        statespace->as<StateSpace>()->writeState(s);

        // check collisions:
        bool inCollision = false;
        for (size_t i = 0; i < task->collisionPairHandles.size() / 2; i++)
        {
            if (task->collisionPairHandles[2 * i + 0] >= 0)
            {
                int r = simCheckCollision(task->collisionPairHandles[2 * i + 0], task->collisionPairHandles[2 * i + 1]);
                if (r > 0)
                {
                    inCollision = true;
                    break;
                }
            }
            if (inCollision)
                break;
        }


        // Check joint states bounds
        bool bound_violated = false;
        for (size_t i = 0; i < g_joint_names.size(); ++i)
        {
            int joint_handle = simGetObjectHandle(g_joint_names[i].c_str());
            float joint_position;
            simGetJointPosition(joint_handle, &joint_position);

            if (joint_position <= g_bounds[i].boundsLow || joint_position >= g_bounds[i].boundsHigh)
            {
                bound_violated = true;
                break;
            }
        }


        // restore original state:
        statespace->as<StateSpace>()->writeState(s_old);
        statespace->as<StateSpace>()->restoreRelPoseState(pose_old);

        return !inCollision && !bound_violated;
    }

    virtual bool checkCallback(const ob::State *state) const
    {
        // ob::CompoundStateSpace *ss = statespace->as<ob::CompoundStateSpace>();
        ob::ScopedState<ob::CompoundStateSpace> s(statespace);
        s = state;

        // save old state:
        ob::ScopedState<ob::CompoundStateSpace> s_old(statespace);
        statespace->as<StateSpace>()->readState(s_old);
        std::vector<float> pose_old;
        statespace->as<StateSpace>()->saveRelPoseState(pose_old);

        // write query state:
        statespace->as<StateSpace>()->writeState(s);

        // check collisions:
        bool inCollision = false;
        for (size_t i = 0; i < task->collisionPairHandles.size() / 2; i++)
        {
            if (task->collisionPairHandles[2 * i + 0] >= 0)
            {
                int r = simCheckCollision(task->collisionPairHandles[2 * i + 0], task->collisionPairHandles[2 * i + 1]);
                if (r > 0)
                {
                    inCollision = true;
                    break;
                }
            }
            if (inCollision)
                break;
        }

        // // now check constrainsts
        if (!inCollision)
        {
            // std::cout << " DO CONSTRAINTS" << std::endl;

            std::vector<float> grasped_item_matrix(16);
            simGetObjectMatrix(g_selected_item_handle, g_dummyWorld_handle, grasped_item_matrix.data());
            Eigen::Matrix3f item_rotation = getRotation(grasped_item_matrix);
            Eigen::Matrix3f item_transform = item_rotation * g_item_transform;

            Eigen::Vector3f a = g_item_transform.col(2).normalized();

            Eigen::Vector3f b = item_rotation.inverse().col(2).normalized();

            float angle = acos(a.dot(b));

            // error appear when a.dot(b) give us 1
            if (std::isnan(angle))
                angle = 0;

            float treshold = M_PI * g_constraint_tolerance / 180;

            // std::cout << "angle is :  " << angle << " treshold is : " << treshold << "\n";

            if (angle < treshold)
                inCollision = false;
            else
                inCollision = true;
        }

        // restore original state:
        statespace->as<StateSpace>()->writeState(s_old);
        statespace->as<StateSpace>()->restoreRelPoseState(pose_old);

        return !inCollision;
    }

    ob::StateSpacePtr statespace;
    TaskDef *task;
};

class Goal : public ob::Goal
{
public:
    Goal(const ob::SpaceInformationPtr &si, TaskDef *task, double tolerance = 1e-3)
        : ob::Goal(si), statespace(si->getStateSpace()), task(task), tolerance(tolerance)
    {
    }

    virtual bool isSatisfied(const ob::State *state) const
    {
        double distance = 0.0;
        return isSatisfied(state, &distance);
    }

    virtual bool isSatisfied(const ob::State *state, double *distance) const
    {
        switch (task->goal.type)
        {
        case TaskDef::Goal::STATE:
            // silence -Wswitch warning
            // if really type is STATE we are not using this class for goal check
            return false;
        case TaskDef::Goal::DUMMY_PAIR:
            return checkDummyPair(state, distance);
        case TaskDef::Goal::CLLBACK:
            return checkCallback(state, distance);
        }

        return false;
    }

protected:
    virtual bool checkDummyPair(const ob::State *state, double *distance) const
    {
        ob::ScopedState<ob::CompoundStateSpace> s(statespace);
        s = state;

        // save old state:
        ob::ScopedState<ob::CompoundStateSpace> s_old(statespace);
        statespace->as<StateSpace>()->readState(s_old);
        std::vector<float> pose_old;
        statespace->as<StateSpace>()->saveRelPoseState(pose_old);

        // write query state:
        statespace->as<StateSpace>()->writeState(s);

        if (task->goal.metric[3] == 0.0)
        { // ignore orientation
            float goalPos[3];
            float robotPos[3];
            simGetObjectPosition(task->goal.dummyPair.goalDummy, task->goal.refDummy, &goalPos[0]);
            simGetObjectPosition(task->goal.dummyPair.robotDummy, task->goal.refDummy, &robotPos[0]);
            *distance = sqrt(pow((goalPos[0] - robotPos[0]) * task->goal.metric[0], 2) +
                             pow((goalPos[1] - robotPos[1]) * task->goal.metric[1], 2) +
                             pow((goalPos[2] - robotPos[2]) * task->goal.metric[2], 2));
        }
        else
        { // do not ignore orientation
            float goalM[12];
            float robotM[12];
            simGetObjectMatrix(task->goal.dummyPair.goalDummy, task->goal.refDummy, goalM);
            simGetObjectMatrix(task->goal.dummyPair.robotDummy, task->goal.refDummy, robotM);
            float axis[3];
            float angle;
            simGetRotationAxis(robotM, goalM, axis, &angle);
            *distance =
                sqrt(pow((goalM[3] - robotM[3]) * task->goal.metric[0], 2) +
                     pow((goalM[7] - robotM[7]) * task->goal.metric[1], 2) +
                     pow((goalM[11] - robotM[11]) * task->goal.metric[2], 2) + pow(angle * task->goal.metric[3], 2));
        }

        bool satisfied = *distance <= tolerance;

        // restore original state:
        statespace->as<StateSpace>()->writeState(s_old);
        statespace->as<StateSpace>()->restoreRelPoseState(pose_old);

        return satisfied;
    }

    virtual bool checkCallback(const ob::State *state, double *distance) const
    {
        // std::vector<double> stateVec;
        // statespace->copyToReals(stateVec, state);

        bool ret = false;

        // goalCallback_in in_args;
        // goalCallback_out out_args;

        // for(size_t i = 0; i < stateVec.size(); i++)
        //     in_args.state.push_back((float)stateVec[i]);

        // if(goalCallback(task->goal.callback.scriptId, task->goal.callback.function.c_str(), &in_args, &out_args))
        // {
        //     ret = out_args.satisfied;
        //     *distance = out_args.distance;
        // }
        // else
        // {
        //     throw ompl::Exception("Goal callback " + task->goal.callback.function + " returned an error");
        // }

        return ret;
    }

    ob::StateSpacePtr statespace;
    TaskDef *task;
    double tolerance;
};

class ValidStateSampler : public ob::UniformValidStateSampler
{
public:
    ValidStateSampler(const ob::SpaceInformation *si, TaskDef *task) : ob::UniformValidStateSampler(si), task(task)
    {
        name_ = "SimValidStateSampler";
    }

    bool sample(ob::State *state)
    {
        // if(task->validStateSampling.type == TaskDef::ValidStateSampling::CLLBACK)
        // {
        //     if(task->validStateSampling.callback.function == "")
        //     {
        //         throw ompl::Exception("Specified empty callback for valid state sampling");
        //     }

        bool ret = false;

        // validStateSamplerCallback_in in_args;
        // validStateSamplerCallback_out out_args;

        // if(validStateSamplerCallback(task->validStateSampling.callback.scriptId,
        // task->validStateSampling.callback.function.c_str(), &in_args, &out_args))
        // {
        //     std::vector<double> stateVec;
        //     for(size_t i = 0; i < out_args.sampledState.size(); i++)
        //         stateVec.push_back((double)out_args.sampledState[i]);
        //     task->stateSpacePtr->copyFromReals(state, stateVec);
        //     ret = true;
        // }
        // else
        // {
        //     throw ompl::Exception("Valid state sampling callback " + task->validStateSampling.callback.function + "
        //     returned an error");
        // }

        return ret;
        // }
        // else
        // {
        //     return ob::UniformValidStateSampler::sample(state);
        // }
    }

    bool sampleNear(ob::State *state, const ob::State *nearState, const double distance)
    {
        // if(task->validStateSampling.type == TaskDef::ValidStateSampling::CLLBACK)
        // {
        //     if(task->validStateSampling.callbackNear.function == "")
        //     {
        //         throw ompl::Exception("Specified empty callback for \"near\" valid state sampling");
        //     }

        //     std::vector<double> nearStateVec;
        //     task->stateSpacePtr->copyToReals(nearStateVec, nearState);

        bool ret = false;

        //     validStateSamplerCallbackNear_in in_args;
        //     validStateSamplerCallbackNear_out out_args;

        //     for(size_t i = 0; i < nearStateVec.size(); i++)
        //         in_args.state.push_back((float)nearStateVec[i]);
        //     in_args.distance = distance;

        //     if(validStateSamplerCallbackNear(task->validStateSampling.callbackNear.scriptId,
        //     task->validStateSampling.callbackNear.function.c_str(), &in_args, &out_args))
        //     {
        //         std::vector<double> stateVec;
        //         for(size_t i = 0; i < out_args.sampledState.size(); i++)
        //             stateVec.push_back((double)out_args.sampledState[i]);
        //         task->stateSpacePtr->copyFromReals(state, stateVec);
        //         ret = true;
        //     }
        //     else
        //     {
        //         throw ompl::Exception("Near valid state sampling callback " +
        //         task->validStateSampling.callbackNear.function + " returned an error");
        //     }

        return ret;
        // }
        // else
        // {
        //     return ob::UniformValidStateSampler::sampleNear(state, nearState, distance);
        // }
    }

protected:
    TaskDef *task;
};

typedef std::shared_ptr<ValidStateSampler> ValidStateSamplerPtr;

ob::ValidStateSamplerPtr allocValidStateSampler(const ob::SpaceInformation *si, TaskDef *task)
{
    return ob::ValidStateSamplerPtr(new ValidStateSampler(si, task));
}

class OutputHandler : public ompl::msg::OutputHandler
{
public:
    void log(const std::string &text, ompl::msg::LogLevel level, const char *filename, int line)
    {
        // int csim_level = sim_verbosity_none;
        // switch(level)
        // {
        // case ompl::msg::LogLevel::LOG_DEV2:
        // case ompl::msg::LogLevel::LOG_DEV1:
        //     csim_level = sim_verbosity_trace;
        //     break;
        // case ompl::msg::LogLevel::LOG_DEBUG:
        //     csim_level = sim_verbosity_debug;
        //     break;
        // case ompl::msg::LogLevel::LOG_INFO:
        //     csim_level = sim_verbosity_infos;
        //     break;
        // case ompl::msg::LogLevel::LOG_WARN:
        //     csim_level = sim_verbosity_warnings;
        //     break;
        // case ompl::msg::LogLevel::LOG_ERROR:
        //     csim_level = sim_verbosity_errors;
        //     break;
        // case ompl::msg::LogLevel::LOG_NONE:
        //     csim_level = sim_verbosity_none;
        //     break;
        // default:
        //     csim_level = sim_verbosity_none;
        //     break;
        // }
        // sim::addLog(csim_level, "OMPL: %s:%d: %s", filename, line, text);
    }
};