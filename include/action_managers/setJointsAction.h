#ifndef SetJointsAction_H
#define SetJointsAction_H

#include "action_managers/staubliControlActionManager.h"

class SetJointsAction: public StaubliControlActionManager
{
   protected:
      ros::NodeHandle nh_;
      actionlib::SimpleActionServer<staubli_tx60::SetJointsAction> as_;
      std::string action_name_;

   public:
      SetJointsAction(std::string name) :
	 as_(nh_, name, boost::bind(&SetJointsAction::setJointsCB, this, _1)),
	 action_name_(name) {}

      bool polling( const std::vector<double> &j1 ) {
	 std::vector<double> j2;
	 j2.resize(6);
	 if(staubli.GetRobotJoints(j2)){
	    feedback_.j = j2;
	    as_.publishFeedback(feedback_);
	    double error = fabs(j1[0]-j2[0])+ fabs(j1[1]-j2[1])+ fabs(j1[2]-j2[2])+
	       fabs(j1[3]-j2[3])+ fabs(j1[4]-j2[4])+ fabs(j1[5]-j2[5]);
	    //	    ROS_INFO( "Error to target % lf", error );
	    return error < ERROR_EPSILON || staubli.IsJointQueueEmpty();
	 }else {
	    ROS_ERROR("Error when determining end of movement.");
	    return false;
	 }
      }

      void setJointsCB( const staubli_tx60::SetJointsGoalConstPtr &goal ) {
	 //staubli.ResetMotion();
	 ros::Rate rate(10);
	 bool success = true;
	 ROS_INFO("Set Joints Action Cmd received \n");
	 if( staubli.MoveJoints(goal->j,
				goal->params.movementType,
				goal->params.jointVelocity,
				goal->params.jointAcc,
				goal->params.jointDec,
				goal->params.endEffectorMaxTranslationVel, 
				goal->params.endEffectorMaxRotationalVel,
				goal->params.distBlendPrev,
				goal->params.distBlendNext
				) )
	   {
	    ROS_INFO("Cmd received, moving to desired joint angles.");
	    while(true){
	       if (as_.isPreemptRequested() || !ros::ok()) {
		  ROS_INFO("%s: Preempted", action_name_.c_str());
		  // set the action state to preempted
		  staubli.ResetMotion();
		  as_.setPreempted();
		  success = false;
		  break;
	       }
	       if( polling(goal->j) ) break;
	       rate.sleep();
	    }
	    if(success) as_.setSucceeded(result_);
	 }else {
	    as_.setAborted();
	    ROS_ERROR("Cannot move to specified joints' configuration.");
	 }
      }
};

#endif /* SetJointsAction_H */
