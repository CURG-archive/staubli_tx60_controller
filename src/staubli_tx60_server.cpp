#include "TX60L.h"
#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include "staubli_tx60/GetCartesian.h"
#include "staubli_tx60/GetJoints.h"
#include "staubli_tx60/GetRotMat.h"
#include "staubli_tx60/FwdKinematics.h"
#include "staubli_tx60/InvKinematics.h"
#include "staubli_tx60/SetJointsAction.h"
#include "staubli_tx60/SetCartesianAction.h"
#include "staubli_tx60/SetJointTrajectoryAction.h"
#include "staubli_tx60/SetJointTrajectoryGoal.h"
#include "staubli_tx60/JointTrajectoryPoint.h"
#include "staubli_tx60/ResetMotion.h"
#include "std_msgs/String.h"
#include <cstring>
#include <vector>
#include <cmath>

#include <boost/foreach.hpp>

#include "sensor_msgs/JointState.h"

#include "servicesManager.h"
#include "staubliJointStatePublisher.h"

#include "action_managers/setCartesianActionManager.h"
#include "action_managers/setJointsActionManager.h"
#include "action_managers/setJointTrajectoryActionManager.h"
#include "action_managers/setGenericJointTrajectoryActionManager.h"


#include "staubliState.h"




const size_t CONTROL_FREQ  = 20;


class StaubliNode
{
    private:
        ros::NodeHandle node_handle;

        TX60L staubli;
        StaubliState currentState;

        std::vector<StaubliActionManager*> actionManagers;
        ServicesManager servicesManager;
        StaubliJointStatePublisher staubliJointStatePublisher;

        ros::Rate loop_rate;

    public:
        StaubliNode();
        void login(std::string url);
        bool isLoggedIn();
        void run();

};


StaubliNode::StaubliNode():
    node_handle(""),
    staubliJointStatePublisher(node_handle,staubli),
    servicesManager(node_handle, staubli),
    loop_rate(10)
{
    SetCartesianActionManager *setCartesianActionManager = new SetCartesianActionManager("setCartesian",&staubli);
    SetJointsActionManager *setJointActionManager = new SetJointsActionManager("setJoints",&staubli);
    SetJointTrajectoryActionManager *setJointTrajectoryActionManager = new SetJointTrajectoryActionManager("setJointTrajectory",&staubli);
    // Same as SetJointTrajectoryActionManager but for the more generic ros trajectory type
    SetGenericJointTrajectoryActionManager *setGenericJointTrajectoryActionManager = new SetGenericJointTrajectoryActionManager("setFollowTrajectory",&staubli);

    actionManagers.push_back(setCartesianActionManager);
    actionManagers.push_back(setJointActionManager);
    actionManagers.push_back(setJointTrajectoryActionManager);
    actionManagers.push_back(setGenericJointTrajectoryActionManager);

    ROS_INFO("* * * * * * * * * * * * * * * * * * * * * * * * *");
    ROS_INFO("* MAKE SURE CS8 CONTROLLER IS IN NETWORKED MODE *");
    ROS_INFO("* * * * * * * * * * * * * * * * * * * * * * * * *");
}

void StaubliNode::login(std::string url)
{
    if(staubli.Login( url, "default", ""))
    {
        ROS_INFO("Connected to Staubli CS8 Controller.");
    }
    else
    {
        ROS_ERROR("Failed to connect to Staubli CS8 Controller.");
    }
}


bool StaubliNode::isLoggedIn()
{
    staubli.IsLoggedIn();
}



void StaubliNode::run()
{
    staubli.Power(true);

    while(ros::ok())
    {
        currentState.updateState(staubli);

        staubliJointStatePublisher.publish(currentState);

        if(!staubli.IsWorking())
        {
           ROS_ERROR("Staubli not logged in or last request failed");
        }


        for(int i =0; i < actionManagers.size(); i++)
        {
            StaubliActionManager *actionManager = actionManagers.at(i);
            if(actionManager->isRunning())
            {
                actionManager->publishFeedback(currentState);
            }
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    staubli.Power(false);
    staubli.Logoff();
}





int main(int argc, char **argv)
{

    if ( argc<2 || (strstr(argv[1],"http://")!=argv[1]) || !strstr(argv[1],":5653/") )
    {
        ROS_ERROR( "Wrong command line arguments\n"
            "Usage: staubli_tx60_server Staubli_CS8_IP\n"
            "Ex:    rosrun staubli_tx60 staubli_tx60_server \"http://192.168.11.xxx:5653/\"");
        return 1;
    }

    ros::init(argc, argv, "staubli_tx60_server");

    StaubliNode staubli_node;

    staubli_node.login(argv[1]);
    if(!staubli_node.isLoggedIn())
    {
        return 255;
    }

    staubli_node.run();

    return 0;
}

