#include "action_managers/setGenericJointTrajectoryActionManager.h"
#include "XmlRpc.h"
#include <boost/foreach.hpp>


void SetGenericJointTrajectoryActionManager::setJointNames()
{
    XmlRpc::XmlRpcValue joint_list;
    joint_list[0]="joint_1";
    joint_list[1]="joint_2";
    joint_list[2]="joint_3";
    joint_list[3]="joint_4";
    joint_list[4]="joint_5";
    joint_list[5]="joint_6";
    if(nh_.hasParam("controller_joint_list"))
        nh_.getParam("controller_joint_list",joint_list);

    for(unsigned int i = 0; i < joint_list.size(); ++i)
    {
        mJointNameToIndexMap[joint_list[i]] = i;
        mJointNames.push_back(joint_list[i]);
    }

}


void SetGenericJointTrajectoryActionManager::setDefaultParameters()
{
    //Create three valid ptrs
    while(mMovementParams.size() < 3)
        mMovementParams.push_back(staubli_tx60::StaubliMovementParamsPtr
                                  (new staubli_tx60::StaubliMovementParams()));

    staubli_tx60::StaubliMovementParams start_traj_params;
    start_traj_params.distBlendNext = 0.01;
    start_traj_params.distBlendPrev = 0.01;
    start_traj_params.jointVelocity = 0.4;
    start_traj_params.jointAcc = 0.04;
    start_traj_params.jointDec = 0.04;
    start_traj_params.endEffectorMaxRotationalVel = 9999.0;
    start_traj_params.endEffectorMaxTranslationVel = 9999.0;
    start_traj_params.movementType = 1;

    *mMovementParams[0] = start_traj_params;

    staubli_tx60::StaubliMovementParams interior_traj_params;
    interior_traj_params = start_traj_params;

    *mMovementParams[1] = start_traj_params;

     staubli_tx60::StaubliMovementParams end_traj_params;
     end_traj_params = start_traj_params;
     end_traj_params.distBlendPrev = 0;
     end_traj_params.distBlendNext = 0;
     end_traj_params.movementType = 0;

     *mMovementParams[2] = end_traj_params;

}




SetGenericJointTrajectoryActionManager::SetGenericJointTrajectoryActionManager(const std::string & actionName, TX60L * st)
    :StaubliControlActionManager<control_msgs::FollowJointTrajectoryAction>(actionName , actionName, st)
{
    mSetParameterServer = nh_.advertiseService("set_trajectory_params", &SetGenericJointTrajectoryActionManager::setTrajectoryParams, this);
    setDefaultParameters();
    setJointNames();
}


bool
SetGenericJointTrajectoryActionManager::setTrajectoryParams(
        staubli_tx60::SetTrajectoryParams::Request &req,
        staubli_tx60::SetTrajectoryParams::Response &res)
{
    if(req.trajectoryPart < mMovementParams.size())
    {
        *mMovementParams[req.trajectoryPart] = req.params;
        res.succeeded = true;
        return true;
    }
    ROS_ERROR("SetGenericTrajectoryActionManager::setTrajectoryParams - Tried to modify movement parameter with index %d, which exceeds the known movement parameter list size",
              req.trajectoryPart);

}



staubli_tx60::SetJointTrajectoryActionGoalPtr
SetGenericJointTrajectoryActionManager::
convertToStaubliJointTrajectory(control_msgs::FollowJointTrajectoryActionGoal & goal)
{
    std::vector<unsigned int> jointIndices;

    BOOST_FOREACH(const std::string joint_name, goal.goal.trajectory.joint_names)
    {
        jointIndices.push_back(mJointNameToIndexMap[joint_name]);
    }

    staubli_tx60::SetJointTrajectoryActionGoalPtr staubliGoalPtr(new staubli_tx60::SetJointTrajectoryActionGoal());
    BOOST_FOREACH(const trajectory_msgs::JointTrajectoryPoint & point, goal.goal.trajectory.points)
    {
        staubli_tx60::JointTrajectoryPoint staubliJointGoal;
        staubliJointGoal.jointValues.resize(mJointNameToIndexMap.size(), 0.0f);
        staubliJointGoal.params = *mMovementParams[1];
        for(unsigned int i = 0; i < jointIndices.size(); ++i)
            staubliJointGoal.jointValues[jointIndices[i]] = point.positions[i];
        staubliGoalPtr->goal.jointTrajectory.push_back(staubliJointGoal);
    }

    if(goal.goal.trajectory.points.size() > 1)
        staubliGoalPtr->goal.jointTrajectory[0].params = *mMovementParams.front();

    staubliGoalPtr->goal.jointTrajectory.back().params = *mMovementParams.back();

    return staubliGoalPtr;
}



bool SetGenericJointTrajectoryActionManager::hasReachedGoal(StaubliState & state)
{
    double error = fabs(mGoalValues[0]-state.currentJoints[0])+ fabs(mGoalValues[1]-state.currentJoints[1])+ fabs(mGoalValues[2]-state.currentJoints[2])+
            fabs(mGoalValues[3]-state.currentJoints[3])+ fabs(mGoalValues[4]-state.currentJoints[4])+ fabs(mGoalValues[5]-state.currentJoints[5]);
    ROS_INFO("SetGenericJointTrajectoryActionManager::hasReachedGoal::Goal error magnitude %f",error);
    return error < ERROR_EPSILON;
}

void SetGenericJointTrajectoryActionManager::updateFeedback(StaubliState & state)
{
        mFeedback.feedback.joint_names = mJointNames;
        mFeedback.feedback.actual.positions = state.currentJoints;
}



void SetGenericJointTrajectoryActionManager::updateResult(StaubliState & state)
{
    if(hasReachedGoal(state))
        mResult.result.error_code = 0;
    else
        mResult.result.error_code = mResult.result.GOAL_TOLERANCE_VIOLATED;

}



bool SetGenericJointTrajectoryActionManager::acceptGoal()
{
    staubli_tx60::SetJointTrajectoryActionGoalConstPtr staubliJointTrajActionGoal = convertToStaubliJointTrajectory(mGoal);
    BOOST_FOREACH(const staubli_tx60::JointTrajectoryPoint &jointGoal, staubliJointTrajActionGoal->goal.jointTrajectory)
    {
        bool goalOk = staubli.MoveJoints(jointGoal.jointValues,
                                jointGoal.params.movementType,
                                jointGoal.params.jointVelocity,
                                jointGoal.params.jointAcc,
                                jointGoal.params.jointDec,
                                jointGoal.params.endEffectorMaxTranslationVel,
                                jointGoal.params.endEffectorMaxRotationalVel,
                                jointGoal.params.distBlendPrev,
                                jointGoal.params.distBlendNext
                                );
        if(!goalOk)
        {
            as_.setAborted(mResult.result, "Could not accept goal\n");
            ROS_INFO("staubli::Goal rejected");
            return false;
        }
    }

    if (staubli.IsJointQueueEmpty() && staubli.IsRobotSettled()){
        ROS_INFO("SetGenericJointTrajectoryActionManager::Goal Not Active!!");
        return false;
    }
    mGoalValues = staubliJointTrajActionGoal->goal.jointTrajectory.back().jointValues;
    return true;
}
