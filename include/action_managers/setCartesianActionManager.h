#ifndef SetCartesianAction_H
#define SetCartesianAction_H

#include "action_managers/staubliControlActionManager.h"
#include "staubli_tx60/SetCartesianAction.h"


const double ERROR_EPSILON = 1E-4;

class SetCartesianActionManager : public StaubliControlActionManager<staubli_tx60::SetCartesianAction,staubli_tx60::SetCartesianActionFeedback,staubli_tx60::SetCartesianActionResult ,staubli_tx60::SetCartesianActionGoal>
{

    public:

        SetCartesianActionManager(const std::string & actionName);

        bool polling( const std::vector<double> &goal );

        void setCartesianCB( const staubli_tx60::SetCartesianGoalConstPtr &goalPtr );
};

#endif /* SetCartesianAction_H */
