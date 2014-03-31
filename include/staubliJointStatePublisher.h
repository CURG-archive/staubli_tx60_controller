#ifndef StaubliJointStatePublisher_H
#define StaubliJointStatePublisher_H

class StaubliJointStatePublisher
{
	private:
		ros::Publisher joints_pub;

	public:
        StaubliJointStatePublisher(ros::NodeHandle node_handle);
        void publish(std::vector<double> lastJointValues);

};

#endif /* StaubliJointStatePublisher_H */
