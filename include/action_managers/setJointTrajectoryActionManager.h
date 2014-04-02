#ifndef SetJointTrajectoryAction_H
#define SetJointTrajectoryAction_H

#include "action_managers/staubliControlActionManager.h"
#include "staubli_tx60/SetJointTrajectoryAction.h"
class SetJointTrajectoryActionManager: public StaubliControlActionManager<staubli_tx60::SetJointTrajectoryAction>
{
  
   public:
      SetJointTrajectoryActionManager(const std::string & actionName,TX60L * st);

      virtual bool acceptGoal();

      virtual void updateFeedback();
      virtual void updateResult();
      virtual bool hasReachedGoal();

};


#endif /* SetJointTrajectoryAction_H */
