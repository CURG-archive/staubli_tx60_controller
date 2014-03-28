class StaublieJointStatePublisher
{
	private:
		ros::Publisher joints_pub;

	public:
		StaublieJointStatePublisher(ros::NodeHandle node_handle;);
		publish(std::vector<double> lastJointValues);

}