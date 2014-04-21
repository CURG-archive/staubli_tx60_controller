#ifndef StaubliJointStatePublisher_H
#define StaubliJointStatePublisher_H

#include "ros/ros.h"
#include "TX60L.h"
#include "staubliState.h"

class StaubliJointStatePublisher
{
	private:
		ros::Publisher joints_pub;
        TX60L staubli;

        std::vector<std::string> jointNames;

	public:
        StaubliJointStatePublisher(ros::NodeHandle node_handle, TX60L _staubli);
        void publish(StaubliState currentState);

};

#endif /* StaubliJointStatePublisher_H */
