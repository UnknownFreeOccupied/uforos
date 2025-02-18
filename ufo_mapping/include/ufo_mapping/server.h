
#ifndef UFOMAPPING_SERVER_H
#define UFOMAPPING_SERVER_H

// UFO
#include <ufo/map/color/map.hpp>
#include <ufo/map/integrator/integrator.hpp>
#include <ufo/map/occupancy/map.hpp>
#include <ufo/map/ufomap.hpp>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace ufo
{
namespace mapping
{
class Server
{
 public:
	Server(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

 private:
	void cloudCallback(sensor_msgs::PointCloud2::ConstPtr const& msg);
	void publishMap(ros::Time timestamp);
	void publishMapVisualization(ros::Time timestamp);

 private:
	// Node handles
	ros::NodeHandle& nh_;
	ros::NodeHandle& nh_priv_;

	// Subscribers
	ros::Subscriber cloud_sub_;

	// Publishers
	ros::Publisher map_pub_;

	ros::Publisher map_vis_pub_;

	// TF2
	tf2_ros::Buffer            tf_buffer_;
	tf2_ros::TransformListener tf_listener_;

	// Map
	ufo::Map3D<OccupancyMap, ColorMap>* map_;
	ufo::Integrator                     integrator_;

	bool  insert_rays_     = false;
	float map_resolution_  = 0.05f;
	int   map_depthlevels_ = 17;

	std::string   frame_id_          = "map";
	ros::Duration transform_timeout_ = ros::Duration().fromSec(0.2);
};
}  // namespace mapping
}  // namespace ufo

#endif