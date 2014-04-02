#ifndef SetCartesianAction_H
#define SetCartesianAction_H

#include "action_managers/staubliControlActionManager.h"
#include "staubli_tx60/SetCartesianAction.h"

class SetCartesianActionManager : public StaubliControlActionManager<staubli_tx60::SetCartesianAction>
{

    private:
        bool invKinematics2( std::vector<double> &targetPos, std::vector<double> & j);


    public:

        SetCartesianActionManager(const std::string & actionName);

        bool polling( const std::vector<double> &goal );

        virtual bool sendGoal();
        virtual void publishFeedback();

};

#endif /* SetCartesianAction_H */
