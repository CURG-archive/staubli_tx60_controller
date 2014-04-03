#ifndef SetJointTrajectoryAction_H
#define SetJointTrajectoryAction_H

#include "action_managers/staubliControlActionManager.h"
#include "staubli_tx60/SetJointTrajectoryAction.h"
class SetJointTrajectoryActionManager: public StaubliControlActionManager<staubli_tx60::SetJointTrajectoryAction>
{
  
   public:
      SetJointTrajectoryActionManager(const std::string & actionName,TX60L * st);

      virtual bool acceptGoal();

      virtual void updateFeedback(StaubliState & state);
      virtual void updateResult(StaubliState & state);
      virtual bool hasReachedGoal(StaubliState & state);

};


#endif /* SetJointTrajectoryAction_H */
