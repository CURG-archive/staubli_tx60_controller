#ifndef SetJointsAction_H
#define SetJointsAction_H

#include "action_managers/staubliControlActionManager.h"
#include "staubli_tx60/SetJointsAction.h"

class SetJointsActionManager: public StaubliControlActionManager<staubli_tx60::SetJointsAction>
{
   public:
      SetJointsActionManager(const std::string & actionName);
      virtual bool acceptGoal();

      virtual void updateFeedback();
      virtual void updateResult();
      virtual bool hasReachedGoal();
};

#endif /* SetJointsAction_H */
