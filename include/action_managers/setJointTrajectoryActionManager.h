#ifndef SetJointTrajectoryAction_H
#define SetJointTrajectoryAction_H

#include "action_managers/staubliControlActionManager.h"
#include "staubli_tx60/SetJointTrajectoryAction.h"
class SetJointTrajectoryActionManager: public StaubliControlActionManager<staubli_tx60::SetJointTrajectoryAction>
{
  
   public:
      SetJointTrajectoryActionManager(const std::string & actionName);
      bool polling( const std::vector<double> &j1 );
      void setJointTrajectoryCB( const staubli_tx60::SetJointTrajectoryGoalConstPtr &goal ) ;


};


#endif /* SetJointTrajectoryAction_H */
