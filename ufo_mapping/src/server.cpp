#include <ufo_mapping/server.h>

// UFO
#include <ufo/cloud/point_cloud.hpp>

// UFOROS
#include <ufo_msgs/Map.h>

#include <ufo_ros/conversions.hpp>

// ROS
#include <visualization_msgs/MarkerArray.h>

namespace ufo
{
namespace mapping
{

Server::Server(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
    : nh_(nh), nh_priv_(nh_priv), tf_listener_(tf_buffer_)
{
	// ROS parameters
	nh_priv_.getParam("map_resolution_m", map_resolution_);
	nh_priv_.getParam("map_depth_levels", map_depthlevels_);
	nh_priv_.getParam("map_frame_id", frame_id_);

	map_ = new Map3D<OccupancyMap, ColorMap>(map_resolution_, map_depthlevels_);

	// Subscribers
	cloud_sub_ = nh_.subscribe("cloud_in", 10, &Server::cloudCallback, this);

	// Publishers
	map_pub_     = nh_priv_.advertise<ufo_msgs::Map>("map", 10);
	map_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("map_vis", 10, true);
}

void Server::cloudCallback(sensor_msgs::PointCloud2::ConstPtr const& msg)
{
	Transform3f transform;
	try {
		transform =
		    ufo_ros::rosToUfo(tf_buffer_
		                          .lookupTransform(frame_id_, msg->header.frame_id,
		                                           msg->header.stamp, transform_timeout_)
		                          .transform);
	} catch (tf2::TransformException& ex) {
		ROS_WARN_THROTTLE(1, "%s", ex.what());
		return;
	}

	PointCloud<3, float, Color> cloud;

	ufo_ros::rosToUfo(*msg, cloud);

	transformInPlace(execution::par, transform, cloud);

	if (insert_rays_) {
		// TODO: enable
		// integrator_.insertRays(execution::par, *map_, cloud, transform.translation, false);
	} else {
		integrator_.insertPoints(execution::par, *map_, cloud, false);
	}

	map_->propagateModified(execution::par);

	publishMapVisualization(msg->header.stamp);
}

void Server::publishMap(ros::Time timestamp)
{
	// TODO: Implement

	// if (0 < map_pub_.getNumSubscribers() || map_pub_.isLatched()) {
	// 	ufo_msgs::Map::Ptr update_msg(new ufo_msgs::Map);
	// 	update_msg->header.stamp    = msg->header.stamp;
	// 	update_msg->header.frame_id = frame_id_;
	// 	update_msg->data            = ufo_msgs::ufoToMsgModified(map_);
	// 	map_pub_.publish(update_msg);
	// }
}

void Server::publishMapVisualization(ros::Time timestamp)
{
	if (map_vis_pub_.getNumSubscribers()) {
		map_->propagateModified();

		visualization_msgs::MarkerArray markers;

		// Clear all existing markers
		visualization_msgs::Marker clear_marker;
		clear_marker.header.frame_id = frame_id_;
		clear_marker.header.stamp    = timestamp;
		clear_marker.action          = visualization_msgs::Marker::DELETEALL;
		markers.markers.push_back(clear_marker);

		visualization_msgs::Marker occupied_marker;
		occupied_marker.header             = clear_marker.header;
		occupied_marker.id                 = 0;
		occupied_marker.ns                 = "occupied";
		occupied_marker.type               = visualization_msgs::Marker::CUBE_LIST;
		occupied_marker.action             = visualization_msgs::Marker::ADD;
		occupied_marker.pose.orientation.w = 1.0;
		occupied_marker.scale.x            = map_->length(0).x;
		occupied_marker.scale.y            = map_->length(0).y;
		occupied_marker.scale.z            = map_->length(0).z;
		occupied_marker.color.a            = 1.0;
		occupied_marker.lifetime           = ros::Duration(0.0);  // Forever
		occupied_marker.frame_locked       = false;

		for (auto node : map_->query(ufo::pred::Leaf() && ufo::pred::Occupied())) {
			auto center = map_->center(node);
			auto color  = map_->color(node);

			geometry_msgs::Point point;
			ufo_ros::ufoToRos(center, point);
			occupied_marker.points.push_back(point);

			std_msgs::ColorRGBA c;
			c.a = 1.0;
			c.b = color.blue / 255.0;
			c.g = color.green / 255.0;
			c.r = color.red / 255.0;
			occupied_marker.colors.push_back(c);
		}

		if (!occupied_marker.points.empty()) {
			markers.markers.push_back(occupied_marker);
		}

		if (!markers.markers.empty()) {
			map_vis_pub_.publish(markers);
		}
	}
}

}  // namespace mapping
}  // namespace ufo