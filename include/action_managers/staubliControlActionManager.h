#ifndef StaubliControlActionManager_H
#define StaubliControlActionManager_H

#include "action_managers/staubliActionManager.h"
#include "actionlib/server/simple_action_server.h"
#include <boost/bind.hpp>
#include <boost/bind/protect.hpp>


template <class ActionSpec>
class StaubliControlActionManager : public StaubliActionManager
{

protected:
    typedef typename actionlib::SimpleActionServer<ActionSpec> ActionSpecServer;
    ActionSpecServer as_;

    typename ActionSpecServer::ActionResult mResult;
    typename ActionSpecServer::ActionFeedback mFeedback;
    typename ActionSpecServer::ActionGoal mGoal;
    std::vector<double> mGoalValues;

    /*@brief abortHard - Abort the current job and shut down the node.
    */
    void abortHard();


    virtual void cancelAction();

    /*@brief pollRobot - Test if the robot has reached its goal
    *
    *@param goal_joints - Vector of goal positions
    *
    *@returns whether the goal is still in progress
    */
    bool pollRobot( const std::vector<double> &goal_joints, StaubliState & state);

    //these are helpers to poll robot that are
    //specific to the subclasses
    virtual void updateFeedback(StaubliState & state) = 0;
    virtual void updateResult(StaubliState & state) = 0;
    virtual bool hasReachedGoal(StaubliState & state) = 0;

    bool isRobotFinishedMoving(){return staubli.IsJointQueueEmpty() && staubli.IsRobotSettled();}


public:
    StaubliControlActionManager(const std::string & actionName, const std::string & actionTopic ,TX60L * st) : StaubliActionManager (actionName, st),
        as_(nh_, actionTopic, false)
    {
        ROS_INFO("StaubliControlActionManager::%s::In constructor",actionTopic.c_str());
        as_.registerGoalCallback(boost::bind(&StaubliControlActionManager::newGoalCallback, this));
        as_.registerPreemptCallback(boost::bind(&StaubliControlActionManager::preemptCallback, this));
        as_.start();
        ROS_INFO("StaubliControlActionManager::%s::Started Action",actionTopic.c_str());
    }

    virtual bool acceptGoal() = 0;


    /*@brief - Callback for recieving a new goal
    * Should cancel any existing actions that are trying to control the robot
    * and send the new goal to the robot
    */
    void newGoalCallback(const typename ActionSpecServer::GoalConstPtr  &goal);
    void newGoalCallback();

    /* @brief runFeedback - Poll the goal to make sure it is still running and legal, send any feedback that needs sending
    * to the actions' clients
    */
    virtual void publishFeedback(StaubliState & state);



};

#include "staubliControlActionManager.hpp"
#endif /* StaubliControlActionManager_H */
