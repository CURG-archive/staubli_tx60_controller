#include "action_managers/setJointTrajectoryActionManager.h"
#include "staubli_tx60/SetJointTrajectoryAction.h"

#include <boost/foreach.hpp>

SetJointTrajectoryActionManager::SetJointTrajectoryActionManager(const std::string & actionName, TX60L * st)
    :StaubliControlActionManager<staubli_tx60::SetJointTrajectoryAction>(actionName , actionName, st)
{

}


void SetJointTrajectoryActionManager::updateFeedback()
{
    std::vector<double> j2;
    j2.resize(6);
    if(staubli.GetRobotJoints(j2))
    {
        mFeedback.feedback.j = j2;
    }
    else
    {
        ROS_ERROR("staubli.GetRobotJoints(j2) failed");
    }
}

void SetJointTrajectoryActionManager::updateResult()
{
    std::vector<double> j2;
    j2.resize(6);
    if(staubli.GetRobotJoints(j2))
    {
        mResult.result.j = j2;
    }
    else
    {
        ROS_ERROR("staubli.GetRobotJoints(j2) failed");
    }
}

bool SetJointTrajectoryActionManager::hasReachedGoal()
{
    std::vector<double> currentJoints;
    std::vector<double> goalJoints = mGoal.goal.jointTrajectory.back().jointValues;
    currentJoints.resize(6);
    staubli.GetRobotJoints(currentJoints);

    double error = fabs(goalJoints[0]-currentJoints[0])+ fabs(goalJoints[1]-currentJoints[1])+ fabs(goalJoints[2]-currentJoints[2])+
            fabs(goalJoints[3]-currentJoints[3])+ fabs(goalJoints[4]-currentJoints[4])+ fabs(goalJoints[5]-currentJoints[5]);

    return error < ERROR_EPSILON;
}

bool SetJointTrajectoryActionManager::acceptGoal()
{
    BOOST_FOREACH(const staubli_tx60::JointTrajectoryPoint &jointGoal, mGoal.goal.jointTrajectory)
    {
        bool goalOk = staubli.MoveJoints(jointGoal.jointValues,
                                jointGoal.params.movementType,
                                jointGoal.params.jointVelocity,
                                jointGoal.params.jointAcc,
                                jointGoal.params.jointDec,
                                jointGoal.params.endEffectorMaxTranslationVel,
                                jointGoal.params.endEffectorMaxRotationalVel,
                                jointGoal.params.distBlendPrev,
                                jointGoal.params.distBlendNext
                                );
        if(!goalOk)
        {
            as_.setAborted(mResult.result, "Could not accept goal\n");
            ROS_INFO("staubli::Goal rejected");
            return false;
        }
    }

    if (!as_.isActive() || staubli.IsJointQueueEmpty()){
        ROS_INFO("Goal Not Active!!");
        return false;
    }

    return true;
}
