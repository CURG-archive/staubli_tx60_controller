#ifndef ServicesManager_H
#define ServicesManager_H

#include "ros/ros.h"
#include "staubli_tx60/GetJointsResponse.h"
#include "TX60L.h"

class ServicesManager
{
private:
	ros::NodeHandle node_handle;
	std::vector<ros::ServiceServer> services;

	//service callbacks
	bool cancelMotionCB(staubli_tx60::ResetMotion::Request & req,
          staubli_tx60::ResetMotion::Response & res);

	bool getCartesianCB(staubli_tx60::GetCartesian::Request  &req,
	      staubli_tx60::GetCartesian::Response &res );

	bool getRotMatCB(staubli_tx60::GetRotMat::Request  &req,
	      staubli_tx60::GetRotMat::Response &res );

	bool getJointsCB(staubli_tx60::GetJoints::Request  &req,
	      staubli_tx60::GetJoints::Response &res );

	bool fwdKinematicsCB(staubli_tx60::FwdKinematics::Request  &req,
	      staubli_tx60::FwdKinematics::Response &res );

	bool invKinematicsCB(staubli_tx60::InvKinematics::Request  &req,
	      staubli_tx60::InvKinematics::Response &res );

	//helper
	bool invKinematics2( std::vector<double> &targetPos,
	      std::vector<double> & j);



public:
    ServicesManager(ros::NodeHandle node_handle);
};


#endif /* ServicesManager_H */
