#include "action_managers/staubliControlActionManager.h"

StaubliControlActionManager::StaubliControlActionManager()
{

}

bool StaubliControlActionManager::abortHard()
{
    //unable to query robot state -- stop robot if possible, this is an important error
    ROS_ERROR(actionName_ + "::Communications Failure! Error when determining end of movement. *****  All goals cancelled and robot queue reset.  Staubli Server shutdown!!!");

    as_.setAborted(result_,"Communications failure - could not query joint position\n");
    //communication failures are bad.
    //Shut down the action server if they occur.  Don't accept new goals.
    as_.shutdown();
    //Cancel existing motion of robot if possible
    staubli.ResetMotion();
    //kill entire server
    ros::shutdown();
    return true;
}

void StaubliControlActionManager::cancelAction()
{
    if(as_.isActive()){
        mResult.status = PREEMPTED;
        staubli.ResetMotion();
        as_.setPreempted(result_,"Action preempted by another action");
    }
}


bool StaubliControlActionManager::pollRobot(const std::vector<double> &goal_joint)
{
    std::vector<double> j2(lastJointValues);
    if(staubli.IsWorking())
    {
        //Calculate feedback
        feedback_.j = j2;
        as_.publishFeedback(feedback_);
        double error = fabs(goal_joints[0]-j2[0])+ fabs(goal_joints[1]-j2[1])+ fabs(goal_joints[2]-j2[2])+
                fabs(goal_joints[3]-j2[3])+ fabs(goal_joints[4]-j2[4])+ fabs(goal_joints[5]-j2[5]);

        //Check if we have stopped moving
        if ( staubli.IsJointQueueEmpty() && staubli.IsRobotSettled())
        {
            result_.j = j2;
            //Check if we are close enough to our goal
            if( error >= ERROR_EPSILON )
            {
                //Something emptied the joint goal queue, but the goal was not reached
                as_.setAborted(result_);
                ROS_WARN(actionName_ + ":: Staubli queue emptied prematurely\n");

            }
            else
            {
                as_.setSucceeded(result_);
                ROS_INFO(actionName_ + "GOAL Reached");
                //Hurray, we have reached our goal!
            }
            return true;
        }
        else // We are still moving and everything is ok
        {
            return false;
        }
    }
    else {
        abortHard();
        return true;
    }
    return true;





void StaubliControlActionManager::newGoalCallback(const ActionType::goal_type_::ConstPtr &goal)
{
    // If there wa sa previous goal of this type, preempt it
    if(as_.isActive())
        as_.setPreempted(result_,"Received new goal");

    goal_ = *goal;
    if(sendGoal())
    {
        as_.acceptNewGoal(goal_);
    }
    // Preempt any other goals and mark this one as running
    activateAction();
}

void StaubliControlActionManager::runFeedback()
{
    if(running)
        if(as_.isActive())
            running = pollRobot(goal_values_);
        else
        {
            //This shouldn't happen
            if(as_.isActive())
            {
                ROS_ERROR(actionName_ + " is active, but not set to running!");
            }
        }
}