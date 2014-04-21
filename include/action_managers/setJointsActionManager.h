#ifndef SetJointsAction_H
#define SetJointsAction_H

#include "action_managers/staubliControlActionManager.h"
#include "staubli_tx60/SetJointsAction.h"

class SetJointsActionManager: public StaubliControlActionManager<staubli_tx60::SetJointsAction>
{
   public:
      SetJointsActionManager(const std::string & actionName, TX60L * st);
      virtual bool acceptGoal();

      virtual void updateFeedback(StaubliState & state);
      virtual void updateResult(StaubliState & state);
      virtual bool hasReachedGoal(StaubliState & state);
};

#endif /* SetJointsAction_H */
