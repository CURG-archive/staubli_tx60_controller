#include "action_managers/setJointsActionManager.h"
#include "staubli_tx60/SetJointsAction.h"

SetJointsActionManager::SetJointsActionManager(const std::string & actionName)
    :StaubliControlActionManager<staubli_tx60::SetJointsAction>(actionName , actionName)
{

}

bool SetJointsActionManager::polling( const std::vector<double> &j1 )
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


void SetJointsActionManager::sendGoal( const staubli_tx60::SetJointsGoalConstPtr &goalPtr ) {

    bool goalOk = staubli.MoveJoints(goalPtr->j,
                           goalPtr->params.movementType,
                           goalPtr->params.jointVelocity,
                           goalPtr->params.jointAcc,
                           goalPtr->params.jointDec,
                           goalPtr->params.endEffectorMaxTranslationVel,
                           goalPtr->params.endEffectorMaxRotationalVel,
                           goalPtr->params.distBlendPrev,
                           goalPtr->params.distBlendNext
                           );
    if(!goalOk)
    {
        as_.setAborted();
        ROS_ERROR("Cannot move to specified joints' configuration.");
    }
}
