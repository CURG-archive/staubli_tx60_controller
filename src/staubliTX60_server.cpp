#include "TX60L.h"
#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include "staubliTX60/GetCartesian.h"
#include "staubliTX60/GetJoints.h"
#include "staubliTX60/GetRotMat.h"
#include "staubliTX60/FwdKinematics.h"
#include "staubliTX60/InvKinematics.h"
#include "staubliTX60/SetJointsAction.h"
#include "staubliTX60/SetCartesianAction.h"
#include "staubliTX60/SetJointTrajectoryAction.h"
#include "staubliTX60/SetJointTrajectoryGoal.h"
#include "staubliTX60/JointTrajectoryPoint.h"
#include "staubliTX60/ResetMotion.h"
#include <cstring>
#include <vector>
#include <cmath>
//
#include "soapCS8ServerV0Proxy.h"
#include "soapCS8ServerV1Proxy.h"
#include "soapCS8ServerV3Proxy.h"
#include "CS8ServerV0.nsmap"

#include <boost/foreach.hpp>






TX60L staubli;

const double ERROR_EPSILON = 1E-4;
const size_t CONTROL_FREQ  = 20;

bool cancelMotion(staubliTX60::ResetMotion::Request & req,
		  staubliTX60::ResetMotion::Response & res){
  staubli.ResetMotion();
  res.succeeded = true;
  return true;
  

}

bool getCartesian(staubliTX60::GetCartesian::Request  &req,
      staubliTX60::GetCartesian::Response &res ) {

   std::vector<double> position;
   position.resize(6);
   if(staubli.GetRobotCartesianPosition(position)){
      res.x  = position[0];
      res.y  = position[1];
      res.z  = position[2];
      res.rx = position[3];
      res.ry = position[4];
      res.rz = position[5];
      ROS_INFO("Robot's Cartesian position sent.");
      //// show rotation matrix of arm base to arm tip
      //ns6__Frame x_fr;
      //staubli.SetFrameFromRxRyRz( &x_fr, position[3], position[4], position[5] );
      //ROS_INFO("%lf, %lf, %lf,", x_fr.nx, x_fr.ox, x_fr.ax);
      //ROS_INFO("%lf, %lf, %lf,", x_fr.ny, x_fr.oy, x_fr.ay);
      //ROS_INFO("%lf, %lf, %lf,", x_fr.nz, x_fr.oz, x_fr.az);
      ////
      return true;
   }else {
      ROS_ERROR("Cannot get Cartesian position.");
      return false;
   }
}

bool getRotMat(staubliTX60::GetRotMat::Request  &req,
      staubliTX60::GetRotMat::Response &res ) {
   std::vector<double> position;
   position.resize(6);
   if(staubli.GetRobotCartesianPosition(position)){
      ns6__Frame fr;
      staubli.SetFrameFromRxRyRz( &fr, position[3], position[4], position[5] );
      res.m.clear();

      res.m.push_back(fr.nx); res.m.push_back(fr.ox); res.m.push_back(fr.ax);
      res.m.push_back(fr.ny); res.m.push_back(fr.oy); res.m.push_back(fr.ay);
      res.m.push_back(fr.nz); res.m.push_back(fr.oz); res.m.push_back(fr.az);

      return true;
   } else {
      ROS_ERROR("Cannot Query Arm's Rotation.");
      return false;
   }
}

bool getJoints(staubliTX60::GetJoints::Request  &req,
      staubliTX60::GetJoints::Response &res ) {
   std::vector<double> joints;
   joints.resize(6);
   if(staubli.GetRobotJoints(joints)){
      res.j = joints;
      ROS_INFO("Robot's joints' configuratoin sent.");
      return true;
   }else {
      ROS_ERROR("Cannot get joints' configuration.");
      return false;
   }
}

bool fwdKinematics(staubliTX60::FwdKinematics::Request  &req,
      staubliTX60::FwdKinematics::Response &res ) {
   std::vector<double> pos;
   if(staubli.ForwardKinematics( req.j, pos )){
      res.x  =  (double) pos[0];
      res.y  =  (double) pos[1];
      res.z  =  (double) pos[2];
      res.rx =  (double) pos[3] / 180.0 * M_PI;
      res.ry =  (double) pos[4] / 180.0 * M_PI;
      res.rz =  (double) pos[5] / 180.0 * M_PI;
      ROS_INFO("Robot's forward kinematics sent." );
      return true;
   }else {
      ROS_ERROR("Cannot get forward kinematics.");
      return false;
   }
}

bool invKinematics2( std::vector<double> &targetPos, 
      std::vector<double> & j) {
   std::vector<double> curJoints;
   j.clear();
   if(staubli.GetRobotJoints( curJoints )){
      if(staubli.InverseKinematics( targetPos, curJoints, j )){
	 if(fabs(j[0])+fabs(j[1])+fabs(j[2])+
	       fabs(j[3])+fabs(j[4])+fabs(j[5]) < ERROR_EPSILON ){
	    // got all zeros... assumes inv kinematics failed
	    // sacrificed some area around the home position
	    ROS_ERROR("Inv kinematics returned all zeros, PROBABLY error.");
	    return false;
	 }
	 return true;
      }
   }
   return false;
}

bool invKinematics(staubliTX60::InvKinematics::Request  &req,
      staubliTX60::InvKinematics::Response &res ) {
   std::vector<double> targetPos, j;
   targetPos.push_back( (double) req.x  );
   targetPos.push_back( (double) req.y  );
   targetPos.push_back( (double) req.z  );
   targetPos.push_back( (double) req.rx );
   targetPos.push_back( (double) req.ry );
   targetPos.push_back( (double) req.rz );
   if( invKinematics2( targetPos, j ) ){ 
      res.j = j;
      ROS_INFO("Robot's inverse kinematics sent." );
      return true;
   } else {
      ROS_ERROR("Cannot get inverse kinematics.");
      return false;
   }
}

class SetJointsAction{
   protected:
      ros::NodeHandle nh_;
      actionlib::SimpleActionServer<staubliTX60::SetJointsAction> as_;
      std::string action_name_;
      // create messages that are used to published feedback/result
      staubliTX60::SetJointsFeedback feedback_;
      staubliTX60::SetJointsResult result_;

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
	    //	    ROS_INFO( "Error to target %lf", error );
	    return error < ERROR_EPSILON || staubli.IsJointQueueEmpty();
	 }else {
	    ROS_ERROR("Error when determining end of movement.");
	    return false;
	 }
      }

      void setJointsCB( const staubliTX60::SetJointsGoalConstPtr &goal ) {
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

class SetJointTrajectoryAction{
   protected:
      ros::NodeHandle nh_;
      actionlib::SimpleActionServer<staubliTX60::SetJointTrajectoryAction> as_;
      std::string action_name_;
      // create messages that are used to published feedback/result
      staubliTX60::SetJointTrajectoryFeedback feedback_;
      staubliTX60::SetJointTrajectoryResult result_;
  
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

      void setJointTrajectoryCB( const staubliTX60::SetJointTrajectoryGoalConstPtr &goal ) 
      {
	 //staubli.ResetMotion();
	 ros::Rate rate(10);
	 bool success = true;
	 //only one goal can be active at a time in this type of server.  Cancel previous motion
	 staubli.ResetMotion();
	 //ROS_ERROR("staubli:: Recieved goal");
	 //For simple action servers, previous goals stop being tracked, the robot shoud
	 BOOST_FOREACH(const staubliTX60::JointTrajectoryPoint &jointGoal,  goal->jointTrajectory){
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

class SetCartesianAction{
   protected:
      ros::NodeHandle nh_;
      actionlib::SimpleActionServer<staubliTX60::SetCartesianAction> as_;
      std::string action_name_;
      // create messages that are used to published feedback/result
      staubliTX60::SetCartesianFeedback feedback_;
      staubliTX60::SetCartesianResult result_;

   public:
      SetCartesianAction(std::string name) :
	 as_(nh_, name, boost::bind(&SetCartesianAction::setCartesianCB, 
		  this, _1)), action_name_(name) {}

      bool polling( const std::vector<double> &goal ) {
	 std::vector<double> now;
	 now.resize(6);
	 if(staubli.GetRobotCartesianPosition(now)){
	    feedback_.x  = (double) now[0];
	    feedback_.y  = (double) now[1];
	    feedback_.z  = (double) now[2];
	    feedback_.rx = (double) now[3];
	    feedback_.ry = (double) now[4];
	    feedback_.rz = (double) now[5];
	    as_.publishFeedback(feedback_);
	    double error = 
	      fabs(goal[0]-now[0])+ fabs(goal[1]-now[1])+ fabs(goal[2]-now[2]);
	    //	   ROS_INFO("GOAL: %f, %f, %f, %f, %f, %f; %f, %f, %f, %f, %f, %f", goal[0], goal[1], goal[2], goal[3], goal[4], goal[5], now[0], now[1], now[2], now[3], now[4], now[5]);
	    ROS_INFO("distance: %f", error);

	    //   fmod( fabs(goal[3]-now[3]), M_PI ) + 
	    //   fmod( fabs(goal[4]-now[4]), M_PI ) + 
	    //   fmod( fabs(goal[5]-now[5]), M_PI ) ;
	    //	    ROS_INFO("SUCCEEDED MOVING: %f, %f", error, ERROR_EPSILON);
	    return error < ERROR_EPSILON || staubli.IsJointQueueEmpty();
	 }else {
	    ROS_ERROR("Error when determining end of movement.");
	    return false;
	 }
      }

      void setCartesianCB( const staubliTX60::SetCartesianGoalConstPtr &goalPtr ) {
	 ros::Rate rate(10);
	 bool success = true;
	 std::vector<double> goal, goalJoints;
	 goal.push_back( (double) goalPtr->x  );
	 goal.push_back( (double) goalPtr->y  );
	 goal.push_back( (double) goalPtr->z  );
	 goal.push_back( (double) goalPtr->rx );
	 goal.push_back( (double) goalPtr->ry );
	 goal.push_back( (double) goalPtr->rz );

	 /*	 ROS_ERROR(" %lf, %lf, %lf",   goalPtr->params.jointVelocity,
		   goalPtr->params.jointAcc,
		   goalPtr->params.jointDec);
	 */
	 if( invKinematics2( goal, goalJoints ) ) { 
	   bool moveOK = false;
	   if( goalPtr -> lineCtrl == 1 )
	     moveOK=staubli.MoveLine  (goal,
				       goalPtr->params.jointVelocity,
				       goalPtr->params.jointAcc,
				       goalPtr->params.jointDec);
	   else
	     moveOK=staubli.MoveJoints(goalJoints,
				       goalPtr->params.movementType,
				       goalPtr->params.jointVelocity,
				       goalPtr->params.jointAcc,
				       goalPtr->params.jointDec);
	   if( moveOK ){
	     ROS_INFO("Cmd received, moving to desired Cartesian pos.");
	     while(true){
	       if (as_.isPreemptRequested() || !ros::ok()) {
		 ROS_INFO("%s: Preempted", action_name_.c_str());
		 // set the action state to preempted
		 staubli.ResetMotion();
		 as_.setPreempted();
		 success = false;
		 break;
	       }
	       if( polling(goal) ){ ROS_INFO("succeeded");  break;}
	       rate.sleep();
	       }
	     if(success) as_.setSucceeded(result_);
	   } else { 
	     as_.setAborted(result_);
	     ROS_ERROR( "Cannot move to specified Cartesian position." );
	   }
	 } else { 
	   as_.setAborted(result_);
	   ROS_ERROR("Cannot get inverse kinematics.");
	 }
      }
};


int main(int argc, char **argv)
{
   if ( argc<2 || (strstr(argv[1],"http://")!=argv[1]) || !strstr(argv[1],":5653/") ) {
      ROS_ERROR( "Wrong command line arguments\n"
	    "Usage: staubliTX60_server Staubli_CS8_IP\n"
	    "Ex:    rosrun staubliTX60 staubliTX60_server \"http://192.168.11.xxx:5653/\"");
      return 1;
   }

   if( staubli.Login( argv[1], "default", "") ){
      staubli.Power(true);
      ROS_INFO("Connected to Staubli CS8 Controller.");
      ros::init(argc, argv, "staubliTX60_server");
      ros::NodeHandle n;

      ros::ServiceServer srv_getCartesian,  srv_getJoints; 
      ros::ServiceServer srv_fwdKinematics, srv_invKinematics; 
      ros::ServiceServer srv_getRotMat;

      if( srv_getCartesian = n.advertiseService( "getCartesian", getCartesian))
	 ROS_INFO("Srv up: get staubli's Cartesian position.");
      if( srv_getJoints    = n.advertiseService("getJoints",    getJoints))
	 ROS_INFO("Srv up: get staubli's joints' configuration.");
      if( srv_getRotMat    = n.advertiseService("getRotMat",    getRotMat))
	 ROS_INFO("Srv up: get staubli's rotation matrix" );
      if( srv_fwdKinematics= n.advertiseService("fwdKinematics", fwdKinematics))
	 ROS_INFO("Srv up: get staubli's forward kinematics.");
      if( srv_invKinematics= n.advertiseService("invKinematics", invKinematics))
	 ROS_INFO("Srv up: get staubli's inverse kinematics.");
      if( srv_invKinematics= n.advertiseService("cancelMotion", cancelMotion))
	 ROS_INFO("Srv up: cancel motion.");
      SetJointsAction setJointsAction   ("setJoints");
      ROS_INFO("Srv up: set staubli's joints' configuration.");
      SetJointTrajectoryAction setJointTrajectoryAction   ("setJointTrajectory");
      ROS_INFO("Srv up: set staubli's joint trajectory.");
      SetCartesianAction setCartesianAction("setCartesian");
      ROS_INFO("Srv up: set staubli's Cartesian pos.");


      ROS_INFO("" );
      ROS_INFO("* * * * * * * * * * * * * * * * * * * * * * * * *");
      ROS_INFO("* MAKE SURE CS8 CONTROLLER IS IN NETWORKED MODE *");
      ROS_INFO("* * * * * * * * * * * * * * * * * * * * * * * * *");

      ros::spin();
      staubli.Power(false);
      staubli.Logoff();
   } else { 
      ROS_ERROR("Failed to connect to Staubli CS8 Controller.");
      return 255; 
   }

   return 0;
}

