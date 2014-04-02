#include "action_managers/staubliControlActionManager.h"

#define ERROR_EPSILON 1E-4

template <class ActionSpec>
void StaubliControlActionManager <ActionSpec>::abortHard()
{
    //unable to query robot state -- stop robot if possible, this is an important error
    ROS_ERROR("%s ::Communications Failure! Error when determining end of movement. *****  All goals cancelled and robot queue reset.  Staubli Server shutdown!!!",actionName_.c_str());

    as_.setAborted(mResult.result,"Communications failure - could not query joint position\n");
    //communication failures are bad.
    //Shut down the action server if they occur.  Don't accept new goals.
    as_.shutdown();
    //Cancel existing motion of robot if possible
    staubli.ResetMotion();
    //kill entire server
    ros::shutdown();
}

template <class ActionSpec>
void StaubliControlActionManager<ActionSpec>::cancelAction()
{
    if(as_.isActive())
    {
        staubli.ResetMotion();
        as_.setPreempted(mResult.result,"Action preempted by another action");
    }
}

template <class ActionSpec>
bool StaubliControlActionManager<ActionSpec>::pollRobot(const std::vector<double> &goal_joints)
{
    if(staubli.IsWorking())
    {
        updateFeedback();
        as_.publishFeedback(mFeedback.feedback);

        if(isRobotFinishedMoving())
        {
            updateResult();

            if(hasReachedGoal())
            {
                as_.setSucceeded(mResult.result);
                ROS_INFO("%s GOAL Reached", actionName_.c_str() );
            }
            else
            {
                as_.setAborted(mResult.result);
                ROS_WARN("%s :: Staubli queue emptied prematurely, but goal was not reached\n",actionName_.c_str());
            }
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        abortHard();
        return false;
    }
    return true;
}


template <class ActionSpec>
void StaubliControlActionManager<ActionSpec>::newGoalCallback(const typename ActionSpecServer::GoalConstPtr  &goal)
{
    // If there wa sa previous goal of this type, preempt it
    if(as_.isActive())
        as_.setPreempted(mResult.result,"Received new goal");

    mGoal.goal = *goal;
    if(acceptGoal())
    {
        as_.acceptNewGoal();
    }
    // Preempt any other goals and mark this one as running
    activateAction();
}

template <class ActionSpec>
void StaubliControlActionManager<ActionSpec>::publishFeedback()
{
    if(running)
    {
        if(as_.isActive())
        {
            running = pollRobot(mGoalValues);
        }
    }
    else
    {
        if(as_.isActive())
        {
            ROS_ERROR("%s is active, but not set to running!", actionName_.c_str() );
        }
    }
}
