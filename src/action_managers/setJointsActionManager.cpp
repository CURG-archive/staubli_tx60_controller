#include "action_managers/setJointsActionManager.h"
#include "staubli_tx60/SetJointsAction.h"

SetJointsActionManager::SetJointsActionManager(const std::string & actionName, TX60L * st)
    :StaubliControlActionManager<staubli_tx60::SetJointsAction>(actionName , actionName, st)
{

}


void SetJointsActionManager::updateFeedback(StaubliState & state)
{

        mFeedback.feedback.j = state.currentJoints;

}

void SetJointsActionManager::updateResult(StaubliState & state)
{    
        mResult.result.j = state.currentJoints;
}

bool SetJointsActionManager::hasReachedGoal(StaubliState & state)
{

    double error = fabs(mGoal.goal.j[0]-state.currentJoints[0])+ fabs(mGoal.goal.j[1]-state.currentJoints[1])+ fabs(mGoal.goal.j[2]-state.currentJoints[2])+
            fabs(mGoal.goal.j[3]-state.currentJoints[3])+ fabs(mGoal.goal.j[4]-state.currentJoints[4])+ fabs(mGoal.goal.j[5]-state.currentJoints[5]);

    return error < ERROR_EPSILON;
}


bool SetJointsActionManager::acceptGoal() {

    bool goalOk = staubli.MoveJoints(mGoal.goal.j,
                           mGoal.goal.params.movementType,
                           mGoal.goal.params.jointVelocity,
                           mGoal.goal.params.jointAcc,
                           mGoal.goal.params.jointDec,
                           mGoal.goal.params.endEffectorMaxTranslationVel,
                           mGoal.goal.params.endEffectorMaxRotationalVel,
                           mGoal.goal.params.distBlendPrev,
                           mGoal.goal.params.distBlendNext
                           );
    if(!goalOk)
    {
        as_.setAborted();
        ROS_ERROR("Cannot move to specified joints' configuration.");
        return false;
    }
    return true;
}
