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




const size_t CONTROL_FREQ  = 20;


class StaubliNode
{
    private:
        ros::NodeHandle node_handle;

        std::vector<double> lastJointValues;
        std::vector<std::string> jointNames;
        TX60L staubli;

        std::vector<StaubliActionManager*> actionManagers;
        ServicesManager servicesManager;
        StaubliJointStatePublisher staubliJointStatePublisher;

        bool loggedIn;

        ros::Rate loop_rate;

    public:
        StaubliNode();
        void login(std::string url);
        bool isLoggedIn();
        void run();

};


StaubliNode::StaubliNode():
    node_handle(""),
    loggedIn(false),
    staubliJointStatePublisher(node_handle,staubli,jointNames),
    servicesManager(node_handle, staubli),
    loop_rate(10)
{
    SetCartesianActionManager *setCartesianActionManager = new SetCartesianActionManager("setCartesian");
    SetJointsActionManager *setJointActionManager = new SetJointsActionManager("setJoints");
    SetJointTrajectoryActionManager *setJointTrajectoryActionManager = new SetJointTrajectoryActionManager("setJointTrajectory");

    actionManagers.push_back(setCartesianActionManager);
    actionManagers.push_back(setJointActionManager);
    actionManagers.push_back(setJointTrajectoryActionManager);

    lastJointValues.assign(6,-1);

    jointNames.push_back("joint_1");
    jointNames.push_back("joint_2");
    jointNames.push_back("joint_3");
    jointNames.push_back("joint_4");
    jointNames.push_back("joint_5");
    jointNames.push_back("joint_6");

    //set joint names
    if(node_handle.hasParam("staubli_joint_names"))
    {
        node_handle.getParam("staubli_joint_names",jointNames);
    }

    ROS_INFO("* * * * * * * * * * * * * * * * * * * * * * * * *");
    ROS_INFO("* MAKE SURE CS8 CONTROLLER IS IN NETWORKED MODE *");
    ROS_INFO("* * * * * * * * * * * * * * * * * * * * * * * * *");
}

void StaubliNode::login(std::string url)
{
    loggedIn = staubli.Login( url, "default", "");
    if(loggedIn)
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
    return loggedIn;
}



void StaubliNode::run()
{
    staubli.Power(true);

    while(ros::ok())
    {
        if(staubli.GetRobotJoints(lastJointValues))
        {
            staubliJointStatePublisher.publish(lastJointValues);
        }

        if(!staubli.IsWorking())
        {
           ROS_ERROR("Staubli not logged in or last request failed");
        }

        //! needs to loop over action server managers and make sure that they are sending feedback if they are running
        //! for(int asmi = 0; asmi < action_server_managers_.size(); ++asmi)
        //!     asmi.sendFeedback(/*lastJointValues*/)

       this->actionManagers.at(0)

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

