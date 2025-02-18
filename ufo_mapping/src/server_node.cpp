
// UFO
#include <ufo_mapping/server.h>

// ROS
#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ufomapping_server_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	ufo::mapping::Server server(nh, nh_priv);

	ros::spin();

	return 0;
}