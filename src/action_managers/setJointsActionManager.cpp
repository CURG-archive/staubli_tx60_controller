#include "action_managers/setJointsActionManager.h"
#include "staubli_tx60/SetJointsAction.h"

SetJointsActionManager::SetJointsActionManager(const std::string & actionName, TX60L * st)
    :StaubliControlActionManager<staubli_tx60::SetJointsAction>(actionName , actionName, st)
{

}


void SetJointsActionManager::updateFeedback()
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

void SetJointsActionManager::updateResult()
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

bool SetJointsActionManager::hasReachedGoal()
{
    std::vector<double> currentJoints;
    std::vector<double> goalJoints = mGoal.goal.j;
    currentJoints.resize(6);
    staubli.GetRobotJoints(currentJoints);

    double error = fabs(goalJoints[0]-currentJoints[0])+ fabs(goalJoints[1]-currentJoints[1])+ fabs(goalJoints[2]-currentJoints[2])+
            fabs(goalJoints[3]-currentJoints[3])+ fabs(goalJoints[4]-currentJoints[4])+ fabs(goalJoints[5]-currentJoints[5]);

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
