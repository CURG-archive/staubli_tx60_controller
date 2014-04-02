#include "action_managers/setJointsActionManager.h"
#include "staubli_tx60/SetJointsAction.h"

SetJointsActionManager::SetJointsActionManager(const std::string & actionName)
    :StaubliControlActionManager<staubli_tx60::SetJointsAction>(actionName , actionName)
{

}

bool SetJointsActionManager::pollRobot( const std::vector<double> &j1 )
{
    std::vector<double> j2;
    j2.resize(6);
    if(staubli.GetRobotJoints(j2))
    {
        mFeedback.feedback.j = j2;
        as_.publishFeedback(mFeedback.feedback);
        double error = fabs(j1[0]-j2[0])+ fabs(j1[1]-j2[1])+ fabs(j1[2]-j2[2])+
                fabs(j1[3]-j2[3])+ fabs(j1[4]-j2[4])+ fabs(j1[5]-j2[5]);

        return error < ERROR_EPSILON || staubli.IsJointQueueEmpty();
    }
    else
    {
        ROS_ERROR("Error when determining end of movement.");
        return false;
    }
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
