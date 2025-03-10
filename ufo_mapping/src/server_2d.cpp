// UFO ROS
#include <ufo_mapping/server_2d.hpp>

// STL
#include <cstddef>
#include <functional>

// ROS
#include <tf2/exceptions.hpp>

namespace ufo_mapping
{
MappingServer<2>::MappingServer(rclcpp::NodeOptions const& options)
    : rclcpp::Node("mapping_server_2d", options)
{
	map_frame_ = this->declare_parameter<std::string>("map_frame", "map");

	tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	insert_points_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
	    "insert_points", 10,
	    [this](sensor_msgs::msg::LaserScan::SharedPtr const msg) { insertPoints(msg); });
	insert_rays_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
	    "insert_rays", 10,
	    [this](sensor_msgs::msg::LaserScan::SharedPtr const msg) { insertRays(msg); });
}

void MappingServer<2>::insertPoints(
    sensor_msgs::msg::LaserScan::SharedPtr const /* msg */)
{
	// TODO: Implement
}

void MappingServer<2>::insertRays(sensor_msgs::msg::LaserScan::SharedPtr const /* msg */)
{
	// auto transform = lookupTransform(map_frame_, msg->header.frame_id,
	// msg->header.stamp); if (!transform.has_value()) { 	return;
	// }

	// auto cloud = ufo_ros::fromMsg<2>(*msg);

	// ufo::Vec2f sensor_origin;

	// inverse_integrator_(map_, cloud, sensor_origin, transform, true);

	// TODO: Implement
}

std::optional<ufo::Transform2f> MappingServer<2>::lookupTransform(
    std::string const& target_frame, std::string const& source_frame,
    rclcpp::Time const& time, rclcpp::Duration const& timeout) const
{
	try {
		auto t = tf_buffer_->lookupTransform(target_frame, source_frame, time, timeout);
		// TODO: Implement
		// return ufo_ros::fromMsg(t.transform);
		return {};
	} catch (tf2::TransformException const& ex) {
		RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
		            source_frame.c_str(), target_frame.c_str(), ex.what());
		return {};
	}
}
}  // namespace ufo_mapping