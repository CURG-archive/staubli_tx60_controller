#ifndef SetJointTrajectoryAction_H
#define SetJointTrajectoryAction_H

#include "action_managers/staubliControlActionManager.h"
#include "staubli_tx60/SetJointTrajectoryAction.h"
class SetJointTrajectoryActionManager: public StaubliControlActionManager<staubli_tx60::SetJointTrajectoryAction>
{
  
   public:
      SetJointTrajectoryActionManager(const std::string & actionName);
      bool pollRobot( const std::vector<double> &j1 );
      virtual bool acceptGoal();



};


#endif /* SetJointTrajectoryAction_H */
