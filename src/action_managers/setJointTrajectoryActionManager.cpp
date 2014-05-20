#include "action_managers/setJointTrajectoryActionManager.h"
#include "staubli_tx60/SetJointTrajectoryAction.h"

#include <boost/foreach.hpp>

SetJointTrajectoryActionManager::SetJointTrajectoryActionManager(const std::string & actionName, TX60L * st)
    :StaubliControlActionManager<staubli_tx60::SetJointTrajectoryAction>(actionName , actionName, st)
{

}


void SetJointTrajectoryActionManager::updateFeedback(StaubliState & state)
{
        mFeedback.feedback.j = state.currentJoints;
}

void SetJointTrajectoryActionManager::updateResult(StaubliState & state)
{

        mResult.result.j =state.currentJoints;

}

bool SetJointTrajectoryActionManager::hasReachedGoal(StaubliState & state)
{
    std::vector<double> & currentJoints(state.currentJoints);
    std::vector<double> & goalJoints(mGoal.goal.jointTrajectory.back().jointValues);

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

    if (staubli.IsJointQueueEmpty()){
        ROS_INFO("SetJointTrajectoryActionManager::Goal Not Active!!");
        return false;
    }

    return true;
}
