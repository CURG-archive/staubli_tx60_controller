class SetCartesianAction{
   protected:
      ros::NodeHandle nh_;
      actionlib::SimpleActionServer<staubli_tx60::SetCartesianAction> as_;
      std::string action_name_;
      // create messages that are used to published feedback/result
      staubli_tx60::SetCartesianFeedback feedback_;
      staubli_tx60::SetCartesianResult result_;

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

      void setCartesianCB( const staubli_tx60::SetCartesianGoalConstPtr &goalPtr ) {
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