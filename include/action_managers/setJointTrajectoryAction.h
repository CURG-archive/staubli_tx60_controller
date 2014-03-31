#ifndef SetJointTrajectoryAction_H
#define SetJointTrajectoryAction_H

#include "action_managers/staubliControlActionManager.h"

class SetJointTrajectoryAction: public StaubliControlActionManager
{
   protected:
      ros::NodeHandle nh_;
      actionlib::SimpleActionServer<staubli_tx60::SetJointTrajectoryAction> as_;
      std::string action_name_;
      // create messages that are used to published feedback/result
      staubli_tx60::SetJointTrajectoryFeedback feedback_;
      staubli_tx60::SetJointTrajectoryResult result_;

  
   public:
      SetJointTrajectoryAction(std::string name) :
	 as_(nh_, name, boost::bind(&SetJointTrajectoryAction::setJointTrajectoryCB, this, _1)),
	 action_name_(name) {}

      bool polling( const std::vector<double> &j1 ) {
	 std::vector<double> j2;
	 j2.resize(6);
	 if(staubli.GetRobotJoints(j2))
	   {
	     feedback_.j = j2;
	     as_.publishFeedback(feedback_);
	     double error = fabs(j1[0]-j2[0])+ fabs(j1[1]-j2[1])+ fabs(j1[2]-j2[2])+
	       fabs(j1[3]-j2[3])+ fabs(j1[4]-j2[4])+ fabs(j1[5]-j2[5]);
	     //	    ROS_INFO( "Error to target %lf", error );
	     if ( staubli.IsJointQueueEmpty() && staubli.IsRobotSettled())
	       {
		 result_.j = j2;
		 if( error >= ERROR_EPSILON )
		   {
		     //Something emptied the joint goal queue, but the goal was not reached
		     as_.setAborted(result_);
		     ROS_WARN("Staubli queue emptied prematurely\n");
		     
		   } else
		   {
		     as_.setSucceeded(result_);
		   }  
		 return true;
	       }else  //if (staubli.IsJointQueueEmpty)
	       {
		 return false;
	      } 
	   } else {//	 if(staubli.GetRobotJoints(j2))
	   //unable to query robot state -- stop robot if possible, this is an important error
	    ROS_ERROR("****SetJointTrajectoryAction:: Communications Failure! Error when determining end of movement. *****  All goals cancelled and robot queue reset.  Staubli Server shutdown!!!");
	    as_.setAborted(result_ ,"Communications failure - could not query joint position\n");	    
	    //communication failures are bad.  
	    //Shut down the action server if they occur.  Don't accept new goals.  
	    as_.shutdown();
	    //Cancel existing motion of robot if possible
	    staubli.ResetMotion();
	    //kill entire server
	    ros::shutdown();
	    return true;
	 }
      }

      void setJointTrajectoryCB( const staubli_tx60::SetJointTrajectoryGoalConstPtr &goal ) 
      {
	 //staubli.ResetMotion();
	 ros::Rate rate(10);
	 bool success = true;
	 //only one goal can be active at a time in this type of server.  Cancel previous motion
	 staubli.ResetMotion();
	 //ROS_ERROR("staubli:: Recieved goal");
	 //For simple action servers, previous goals stop being tracked, the robot shoud
	 BOOST_FOREACH(const staubli_tx60::JointTrajectoryPoint &jointGoal,  goal->jointTrajectory){
	   //ROS_ERROR("movementType: %d, jointV: %lf, acc: %lf, dacc: %lf", jointGoal.params.jointVelocity, jointGoal.params.movementType, jointGoal.params.jointAcc, jointGoal.params.jointDec);
	   /*	   if( !staubli.MoveJoints(jointGoal.jointValues,
				   jointGoal.params.movementType,
				   0.4,//jointGoal.params.jointVelocity,
				   0.04,//jointGoal.params.jointAcc, 
				   0.04,//jointGoal.params.jointDec, 
				   jointGoal.params.endEffectorMaxTranslationVel, 
	                           jointGoal.params.endEffectorMaxRotationalVel,
	                           jointGoal.params.distBlendPrev,
                        	   jointGoal.params.distBlendNext
	       ))	    
	   */
	   if( !staubli.MoveJoints(jointGoal.jointValues,
				   jointGoal.params.movementType,
				   jointGoal.params.jointVelocity,
				   jointGoal.params.jointAcc, 
				   jointGoal.params.jointDec, 
				   jointGoal.params.endEffectorMaxTranslationVel, 
				   jointGoal.params.endEffectorMaxRotationalVel,
	                           jointGoal.params.distBlendPrev,
                        	   jointGoal.params.distBlendNext
	       ))
	     {
	     //couldn't accept joint goal
	     as_.setAborted(result_, "Could not accept goal\n");
	     ROS_INFO("staubli::Goal rejected");
	     return;
	   }
	   ROS_INFO("Dist Blend Next: %f\n", jointGoal.params.distBlendNext);
	 }
	 if (!as_.isActive() || staubli.IsJointQueueEmpty()){
	   ROS_INFO("Goal Not Active!!");
	   return;
	 }

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
	   if( polling(goal->jointTrajectory.back().jointValues) ) break;
	   rate.sleep();
	 }
	 if(success)
	   as_.setSucceeded(result_);
	 ROS_INFO("staubli::Goal ended");
      }

};


#endif /* SetJointTrajectoryAction_H */