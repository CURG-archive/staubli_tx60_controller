#ifndef SetJointTrajectoryAction_H
#define SetJointTrajectoryAction_H

#include "action_managers/staubliControlActionManager.h"
#include "staubli_tx60/SetJointTrajectoryAction.h"
class SetJointTrajectoryActionManager: public StaubliControlActionManager<staubli_tx60::SetJointTrajectoryAction>
{
  
   public:
      SetJointTrajectoryActionManager(const std::string & actionName);
      bool polling( const std::vector<double> &j1 );
      virtual bool sendGoal() ;

      virtual void publishFeedback();


};


#endif /* SetJointTrajectoryAction_H */
