#ifndef StaubliControlActionManager_H
#define StaubliControlActionManager_H

#include "action_managers/staubliActionManager.h"
#include "actionlib/server/simple_action_server.h"
#include <boost/bind.hpp>
#include <boost/bind/protect.hpp>
//#include <actionlib/action_definition.h>


template <class ActionSpec>
class StaubliControlActionManager : public StaubliActionManager
{
    //ACTION_DEFINITION(ActionSpec);

protected:
    typedef typename actionlib::SimpleActionServer<ActionSpec> ActionSpecServer;
    ActionSpecServer as_;

    typename ActionSpecServer::ActionResult mResult;
    typename ActionSpecServer::ActionFeedback mFeedback;
    typename ActionSpecServer::ActionGoal mGoal;
    std::vector<double> mGoalValues;

    /*@brief abortHard - Abort the current job and shut down the node.
    *
    *
    */
    void abortHard();


    virtual void cancelAction();

    /*@brief pollRobot - Test if the robot has reached its goal
    *
    *@param goal_joints - Vector of goal positions
    *
    *@returns whether the goal is still in progress
    */
    bool pollRobot( const std::vector<double> &goal_joints) ;


public:
    StaubliControlActionManager(const std::string & actionName, const std::string & actionTopic) : StaubliActionManager (actionName),
        as_(nh_, actionTopic, boost::bind(&StaubliControlActionManager::newGoalCallback, this, _1))
    {
    }

    //virtual void sendGoal() = 0;


    /*@brief - Callback for recieving a new goal
    * Should cancel any existing actions that are trying to control the robot
    * and send the new goal to the robot
    */
    void newGoalCallback(const typename ActionSpecServer::GoalConstPtr  &goal);


    /* @brief runFeedback - Poll the goal to make sure it is still running and legal, send any feedback that needs sending
    * to the actions' clients
    */
    void runFeedback();

};

#include "staubliControlActionManager.hpp"
#endif /* StaubliControlActionManager_H */
