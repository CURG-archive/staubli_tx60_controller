
#include "staubliJointStatePublisher.h"
#include "sensor_msgs/JointState.h"

StaubliJointStatePublisher::StaubliJointStatePublisher(ros::NodeHandle node_handle, TX60L _staubli, std::vector<std::string> _jointNames):
    staubli(_staubli),
    jointNames(_jointNames)
{
    joints_pub = node_handle.advertise<sensor_msgs::JointState>("joints", 10);
    lastJointValues.assign(6,-1);
}

void StaubliJointStatePublisher::publish()
{
	sensor_msgs::JointState jointStateMsg;
	if(staubli.GetRobotJoints(lastJointValues))
	{
	   jointStateMsg.name = jointNames;
	   jointStateMsg.position = lastJointValues;
	   joints_pub.publish(jointStateMsg);
	}
}
