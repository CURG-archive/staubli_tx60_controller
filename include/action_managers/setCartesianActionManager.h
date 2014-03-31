#ifndef SetCartesianAction_H
#define SetCartesianAction_H

#include "action_managers/staubliControlActionManager.h"
#include "staubli_tx60/SetCartesianAction.h"

class SetCartesianActionManager : public StaubliControlActionManager<staubli_tx60::SetCartesianAction,staubli_tx60::SetCartesianActionFeedback,staubli_tx60::SetCartesianActionResult ,staubli_tx60::SetCartesianActionGoal>
{

    private:
        bool invKinematics2( std::vector<double> &targetPos, std::vector<double> & j);


    public:

        SetCartesianActionManager(const std::string & actionName);

        bool polling( const std::vector<double> &goal );

        void newGoalCallbackCB( const staubli_tx60::SetCartesianGoalConstPtr &goalPtr );

};

#endif /* SetCartesianAction_H */
