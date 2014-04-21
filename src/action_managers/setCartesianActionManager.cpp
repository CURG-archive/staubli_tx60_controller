#include "action_managers/setCartesianActionManager.h"


SetCartesianActionManager::SetCartesianActionManager(const std::string & actionName, TX60L * st)
    :StaubliControlActionManager<staubli_tx60::SetCartesianAction>(actionName , actionName, st)
{

}


void SetCartesianActionManager::updateFeedback(StaubliState & state)
{
    std::vector<double> now;
    now.resize(6);
    if(staubli.GetRobotCartesianPosition(now))
    {
        mFeedback.feedback.x  = (double) state.currentCartesianPosition[0];
        mFeedback.feedback.y  = (double) state.currentCartesianPosition[1];
        mFeedback.feedback.z  = (double) state.currentCartesianPosition[2];
        mFeedback.feedback.rx = (double) state.currentCartesianPosition[3];
        mFeedback.feedback.ry = (double) state.currentCartesianPosition[4];
        mFeedback.feedback.rz = (double) state.currentCartesianPosition[5];

    }
    else
    {
        ROS_ERROR("staubli.GetRobotCartesianPosition(now) failed");
    }
}

void SetCartesianActionManager::updateResult(StaubliState & state)
{
        mResult.result.x  = (double) state.currentCartesianPosition[0];
        mResult.result.y  = (double) state.currentCartesianPosition[1];
        mResult.result.z  = (double) state.currentCartesianPosition[2];
        mResult.result.rx = (double) state.currentCartesianPosition[3];
        mResult.result.ry = (double) state.currentCartesianPosition[4];
        mResult.result.rz = (double) state.currentCartesianPosition[5];
}

bool SetCartesianActionManager::hasReachedGoal(StaubliState & state)
{
    // FIXME: Shouldn't this be the euclidean norm?
    double error = fabs(mGoalValues[0]-state.currentCartesianPosition[0]) +
            fabs(mGoalValues[1]-state.currentCartesianPosition[1])+
                   fabs(mGoalValues[2]-state.currentCartesianPosition[2]);

    return (error < ERROR_EPSILON);
}

bool SetCartesianActionManager::acceptGoal()
 {
    std::vector<double> goal, goalJoints;
    mGoalValues.push_back( (double) mGoal.goal.x  );
    mGoalValues.push_back( (double) mGoal.goal.y  );
    mGoalValues.push_back( (double) mGoal.goal.z  );
    mGoalValues.push_back( (double) mGoal.goal.rx );
    mGoalValues.push_back( (double) mGoal.goal.ry );
    mGoalValues.push_back( (double) mGoal.goal.rz );

    if( invKinematics2( goal, goalJoints ) )
    {
        bool moveOK = false;
        if( mGoal.goal.lineCtrl == 1 )
        {
            moveOK=staubli.MoveLine(mGoalValues,
                                      mGoal.goal.params.jointVelocity,
                                      mGoal.goal.params.jointAcc,
                                      mGoal.goal.params.jointDec);
        }
        else
        {
            moveOK=staubli.MoveJoints(goalJoints,
                                      mGoal.goal.params.movementType,
                                      mGoal.goal.params.jointVelocity,
                                      mGoal.goal.params.jointAcc,
                                      mGoal.goal.params.jointDec);
        }

        if(!moveOK )
        {
            as_.setAborted(mResult.result);
            ROS_ERROR( "Cannot move to specified Cartesian position." );
            return false;
        }
    }
    else
    {
        as_.setAborted(mResult.result);
        ROS_ERROR("Cannot get inverse kinematics.");
        return false;
    }
    return true;
}

bool SetCartesianActionManager::invKinematics2( std::vector<double> &targetPos,
      std::vector<double> & j)
{
  std::vector<double> curJoints;
  j.clear();
  if(staubli.GetRobotJoints( curJoints ))
  {
    if(staubli.InverseKinematics( targetPos, curJoints, j ))
    {
      if(fabs(j[0])+fabs(j[1])+fabs(j[2])+
      fabs(j[3])+fabs(j[4])+fabs(j[5]) < ERROR_EPSILON )
      {
        ROS_ERROR("Inv kinematics returned all zeros, PROBABLY error.");
        return false;
      }
      return true;
    }
  }
  return false;
}

