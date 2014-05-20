#ifndef StaubliActionManager_H
#define StaubliActionManager_H

#include <ros/node_handle.h>
#include "TX60L.h"
#include "std_msgs/String.h"
#include "staubliState.h"

class StaubliActionManager
{
protected:
    ros::Publisher activeActionCancelledPub;
    ros::Subscriber activeActionCancelledSub;
    ros::NodeHandle nh_;
    std::string actionName_;

    TX60L & staubli;

    bool running;
    /*@brief activate the current action - set to running and cancel everyone else
     *
     */
    void activateAction()
    {
        std_msgs::StringPtr strMsg (new std_msgs::String);
        strMsg->data = actionName_;
        activeActionCancelledPub.publish(strMsg);
        running = true;
    }

public:
    StaubliActionManager(const std::string & name, TX60L * st) : actionName_(name), staubli(*st)
    {
        activeActionCancelledPub = nh_.advertise<std_msgs::String>("StaubliActionCancelled", 10 );
        activeActionCancelledSub = nh_.subscribe("StaubliActionCancelled",10, &StaubliActionManager::actionCancelledCB, this);
    }


    virtual void actionCancelledCB(const std_msgs::StringConstPtr &msg){

        if(running && actionName_.compare(msg->data))
        {
            ROS_WARN("Staubli::%s::Cancelled action with message %s",actionName_.c_str(), msg->data.c_str());
            cancelAction();
            running = false;
        }
    }

    bool isRunning()
    {
        return running;
    }

    virtual void cancelAction() = 0;

    virtual void preemptCallback()
    {
        ROS_WARN("Staubli::%s::In preempt callback",actionName_.c_str());
        cancelAction();
    }

    virtual void publishFeedback(StaubliState & state) = 0;
};

#endif /* StaubliActionManager_H */
