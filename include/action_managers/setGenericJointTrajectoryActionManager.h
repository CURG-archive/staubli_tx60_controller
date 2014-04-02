#ifndef SETGENERICJOINTTRAJECTORYACTIONMANAGER_H
#define SETGENERICJOINTTRAJECTORYACTIONMANAGER_H

#include "action_managers/staubliControlActionManager.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "staubli_tx60/SetTrajectoryParams.h"
#include "ros/service.h"
#include <vector>
#include <string>
#include <map>
#include "staubli_tx60/SetJointTrajectoryActionGoal.h"

class SetGenericJointTrajectoryActionManager: public StaubliControlActionManager<control_msgs::FollowJointTrajectoryAction>
{

   public:
      SetJointTrajectoryActionManager(const std::string & actionName);
      bool polling( const std::vector<double> &j1 );
      virtual bool sendGoal() ;

      virtual void publishFeedback();
      bool setTrajectoryParams(staubli_tx60::SetTrajectoryParams::Request  &req,
                               staubli_tx60::SetTrajectoryParams::Response &res);
      ros::ServiceServer mSetParameterServer;
      std::vector<staubli_tx60::StaubliMovementParamsPtr> mMovementParams;

      void setDefaultParameters();
      std::map<std::string, unsigned int> mJointNameToIndexMap;

private:
      staubli_tx60::SetJointTrajectoryActionGoalPtr
      convertToStaubliJointTrajectory(control_msgs::FollowJointTrajectoryActionGoal);
};


#endif // SETGENERICJOINTTRAJECTORYACTIONMANAGER_H
