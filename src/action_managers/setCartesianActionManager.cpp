#include "action_managers/setCartesianActionManager.h"


SetCartesianActionManager::SetCartesianActionManager(const std::string & actionName)
    :StaubliControlActionManager<staubli_tx60::SetCartesianAction>(actionName , actionName)
{

}


bool SetCartesianActionManager::polling( const std::vector<double> &goal)
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

void SetCartesianActionManager::newGoalCallbackCB(const staubli_tx60::SetCartesianGoalConstPtr &goalPtr )
 {
    ros::Rate rate(10);
    std::vector<double> goal, goalJoints;
    mGoalValues.push_back( (double) goalPtr->x  );
    mGoalValues.push_back( (double) goalPtr->y  );
    mGoalValues.push_back( (double) goalPtr->z  );
    mGoalValues.push_back( (double) goalPtr->rx );
    mGoalValues.push_back( (double) goalPtr->ry );
    mGoalValues.push_back( (double) goalPtr->rz );

    if( invKinematics2( goal, goalJoints ) )
    {
        bool moveOK = false;
        if( goalPtr -> lineCtrl == 1 )
        {
            moveOK=staubli.MoveLine(mGoalValues,
                                      goalPtr->params.jointVelocity,
                                      goalPtr->params.jointAcc,
                                      goalPtr->params.jointDec);
        }
        else
        {
            moveOK=staubli.MoveJoints(goalJoints,
                                      goalPtr->params.movementType,
                                      goalPtr->params.jointVelocity,
                                      goalPtr->params.jointAcc,
                                      goalPtr->params.jointDec);
        }

        if( moveOK )
        {
            ROS_INFO("Cmd received, moving to desired Cartesian pos.");
            while(true)
            {
                if (as_.isPreemptRequested() || !ros::ok())
                {
                    ROS_INFO("%s: Preempted", actionName_.c_str());
                    staubli.ResetMotion();
                    as_.setPreempted();
                    break;
                }
                if( polling(goal) )
                {
                    ROS_INFO("succeeded");
                    as_.setSucceeded(mResult.result);
                    break;
                }
                rate.sleep();
            }
        }
        else
        {
            as_.setAborted(mResult.result);
            ROS_ERROR( "Cannot move to specified Cartesian position." );
        }
    }
    else
    {
        as_.setAborted(mResult.result);
        ROS_ERROR("Cannot get inverse kinematics.");
    }
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

