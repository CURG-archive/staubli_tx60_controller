#include "action_managers/setCartesianActionManager.h"


SetCartesianActionManager::SetCartesianActionManager(const std::string & actionName)
    :StaubliControlActionManager<staubli_tx60::SetCartesianAction>(actionName , actionName)
{

}


bool SetCartesianActionManager::pollRobot( const std::vector<double> &goal)
{
    std::vector<double> now;
    now.resize(6);
    if(staubli.GetRobotCartesianPosition(now))
    {
        mFeedback.feedback.x  = (double) now[0];
        mFeedback.feedback.y  = (double) now[1];
        mFeedback.feedback.z  = (double) now[2];
        mFeedback.feedback.rx = (double) now[3];
        mFeedback.feedback.ry = (double) now[4];
        mFeedback.feedback.rz = (double) now[5];
        as_.publishFeedback(mFeedback.feedback);
        double error = fabs(goal[0]-now[0])+ fabs(goal[1]-now[1])+ fabs(goal[2]-now[2]);

        ROS_INFO("distance: %f", error);

        return (error < ERROR_EPSILON) || staubli.IsJointQueueEmpty();
    }
    else
    {
        ROS_ERROR("Error when determining end of movement.");
        return false;
    }
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

