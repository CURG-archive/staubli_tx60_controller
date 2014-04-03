
#include "staubliJointStatePublisher.h"
#include "sensor_msgs/JointState.h"

StaubliJointStatePublisher::StaubliJointStatePublisher(ros::NodeHandle node_handle, TX60L _staubli):
    staubli(_staubli)
{
    joints_pub = node_handle.advertise<sensor_msgs::JointState>("joint_states", 10);

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
}

void StaubliJointStatePublisher::publish(StaubliState currentState)
{
  sensor_msgs::JointStatePtr jointStateMsg(new sensor_msgs::JointState());
    jointStateMsg->name = jointNames;
    jointStateMsg->position = currentState.currentJoints;
    jointStateMsg->header.stamp = ros::Time::now();
    joints_pub.publish(jointStateMsg);
}
