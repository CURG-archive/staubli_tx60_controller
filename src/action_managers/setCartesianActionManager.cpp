#include "action_managers/setCartesianActionManager.h"



SetCartesianActionManager::SetCartesianActionManager(const std::string & actionName)
:StaubliControlActionManager(actionName , actionName)
{

}


bool SetCartesianActionManager::polling( const std::vector<double> &goal)
{
    std::vector<double> now;
    now.resize(6);
    if(staubli.GetRobotCartesianPosition(now))
    {
        mFeedback_.feedback.x  = (double) now[0];
        mFeedback_.feedback.y  = (double) now[1];
        mFeedback_.feedback.z  = (double) now[2];
        mFeedback_.feedback.rx = (double) now[3];
        mFeedback_.feedback.ry = (double) now[4];
        mFeedback_.feedback.rz = (double) now[5];
        as_.publishFeedback(mFeedback_);
        double error = fabs(goal[0]-now[0])+ fabs(goal[1]-now[1])+ fabs(goal[2]-now[2]);

        ROS_INFO("distance: %f", error);

        return error < ERROR_EPSILON || staubli.IsJointQueueEmpty();
    }
    else
    {
        ROS_ERROR("Error when determining end of movement.");
        return false;
    }
}

void SetCartesianActionManager::setCartesianCB( const staubli_tx60::SetCartesianGoalConstPtr &goalPtr )
 {
    ros::Rate rate(10);
    std::vector<double> goal, goalJoints;
    mGoal.push_back( (double) goalPtr->x  );
    mGoal.push_back( (double) goalPtr->y  );
    mGoal.push_back( (double) goalPtr->z  );
    mGoal.push_back( (double) goalPtr->rx );
    mGoal.push_back( (double) goalPtr->ry );
    mGoal.push_back( (double) goalPtr->rz );

    if( invKinematics2( goal, goalJoints ) )
    {
        bool moveOK = false;
        if( goalPtr -> lineCtrl == 1 )
        {

            moveOK=staubli.MoveLine(mGoal,
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
                    as_.setSucceeded(mResult);
                    break;
                }
                rate.sleep();
            }
        }
        else
        {
            as_.setAborted(mResult);
            ROS_ERROR( "Cannot move to specified Cartesian position." );
        }
    }
    else
    {
        as_.setAborted(mResult);
        ROS_ERROR("Cannot get inverse kinematics.");
    }
}
