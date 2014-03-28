StaubliJointStatePublisher::StaubliJointStatePublisher(ros::NodeHandle node_handle;)
{
    joints_pub = node_handle.advertise<sensor_msgs::JointState>("joints", 10);
}

StaubliJointStatePublisher::publish(std::vector<double> lastJointValues)
{
	sensor_msgs::JointState jointStateMsg;
	if(staubli.GetRobotJoints(lastJointValues))
	{
	   jointStateMsg.name = jointNames;
	   jointStateMsg.position = lastJointValues;
	   joints_pub.publish(jointStateMsg);
	}
}