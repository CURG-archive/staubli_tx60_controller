#ifndef SetJointsAction_H
#define SetJointsAction_H

#include "action_managers/staubliControlActionManager.h"
#include "staubli_tx60/SetJointsAction.h"

class SetJointsActionManager: public StaubliControlActionManager<staubli_tx60::SetJointsAction>
{
   public:
      SetJointsActionManager(const std::string & actionName);
      bool polling( const std::vector<double> &j1 );
      void setJointsCB( const staubli_tx60::SetJointsGoalConstPtr &goalPtr );
};

#endif /* SetJointsAction_H */
