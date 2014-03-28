class StaubliActionManager
{
protected:
    ros::Publisher activeActionCancelledPub;
    ros::Subscriber activeActionCancelledSub;
    ros::NodeHandle nh_;
    std::string actionName_;
    bool running;
    /*@brief activate the current action - set to running and cancel everyone else
     *
     */
    void activateAction()
    {
        activeActionCancelledPub.publish(actionName_);
        running = true;
    }

public:
    StaubliActionManager(std::string & name) : name_(name)
    {
        activeActionCancelledPub = nh_.advertise<std_msgs::String>("StaubliActionCancelled", 10 );
        activeActionCancelledSub = nh_.subscribe("StaubliActionCancelled", actionCancelledCB, 10);
    }


    virtual void actionCancelledCB(std_msgs::StringConstPtr & msg){

        if(running && !actionName_.compare(msg->data))
        {
            cancelAction();
            running = false;
        }
    }
    virtual void cancelAction() = 0;
};
