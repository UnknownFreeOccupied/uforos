
#ifndef UFOMAPPING_SERVER_H
#define UFOMAPPING_SERVER_H

// UFO
#include <ufo/map/color/map.hpp>
// #include <ufo/map/integrator/simple_integrator.hpp>
#include <ufo/map/integrator/inverse_integrator.hpp>
// #include <ufo/map/integrator/point_integrator.hpp>
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
	void insertHits(sensor_msgs::PointCloud2::ConstPtr const& msg);
	void insertRays(sensor_msgs::PointCloud2::ConstPtr const& msg);
	void publishMap(ros::Time timestamp);
	void publishMapVisualization(ros::Time timestamp);

 private:
	// Subscribers
	ros::Subscriber insert_hits_sub_;
	ros::Subscriber insert_rays_sub_;

	// Publishers
	ros::Publisher map_pub_;

	ros::Publisher map_vis_pub_;

	// TF2
	tf2_ros::Buffer            tf_buffer_;
	tf2_ros::TransformListener tf_listener_;

	// Map
	ufo::Map3D<OccupancyMap, ColorMap>* map_;
	// ufo::Integrator                     integrator_;
	// ufo::SimpleIntegrator<3> simple_integrator_;
	ufo::InverseIntegrator<3> inverse_integrator_;
	// ufo::PointIntegrator<3> point_integrator_;

	float map_resolution_  = 0.05f;
	int   map_depthlevels_ = 17;

	std::string   frame_id_          = "map";
	ros::Duration transform_timeout_ = ros::Duration().fromSec(0.2);
};
}  // namespace mapping
}  // namespace ufo

#endif