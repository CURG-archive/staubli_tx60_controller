#ifndef StaubliJointStatePublisher_H
#define StaubliJointStatePublisher_H

#include "ros/ros.h"
#include "TX60L.h"

class StaubliJointStatePublisher
{
	private:
		ros::Publisher joints_pub;
        TX60L staubli;

        std::vector<std::string> jointNames;
        std::vector<double> lastJointValues;

	public:
        StaubliJointStatePublisher(ros::NodeHandle node_handle, TX60L _staubli, std::vector<std::string> jointNames);
        void publish();

};

#endif /* StaubliJointStatePublisher_H */
