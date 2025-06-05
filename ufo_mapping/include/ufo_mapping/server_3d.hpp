#ifndef UFO_MAPPING_SERVER_3D_HPP
#define UFO_MAPPING_SERVER_3D_HPP

// UFO
#include <ufo/map/integrator/angular_integrator.hpp>
// #include <ufo/map/integrator/inverse_integrator.hpp>
// #include <ufo/map/integrator/simple_integrator.hpp>
#include <ufo/map/ufomap.hpp>

// UFO ROS
#include <ufo_interfaces/msg/map.hpp>
#include <ufo_mapping/detail/server.hpp>
#include <ufo_ros/ufo_ros.hpp>

// STL
#include <future>
#include <memory>
#include <optional>

// ROS
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace ufo_mapping
{
template <>
class MappingServer<3> : public rclcpp::Node
{
 public:
	/**
	 * @brief A constructor for ufo_mapping::MappingServer
	 * @param options Additional options to control creation of the node.
	 */
	MappingServer(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

 private:
	void insert(sensor_msgs::msg::PointCloud2::SharedPtr const msg);

	void insert(ufo::PointCloud<3, float, ufo::Color> const& cloud,
	            ufo::Transform<3, float> const& transform, rclcpp::Time const& time);

	[[nodiscard]] std::optional<ufo::Transform3f> lookupTransform(
	    std::string const& target_frame, std::string const& source_frame,
	    rclcpp::Time const&     time,
	    rclcpp::Duration const& timeout = rclcpp::Duration::from_nanoseconds(0)) const;

	void publishUpdate(rclcpp::Time const& time) const;

 private:
	ufo::Map3D<ufo::OccupancyMap, ufo::ColorMap, ufo::VoidRegionMap> map_;
	// ufo::MapFull3D<ufo::MapUtility::WITH_CENTER, ufo::OccupancyMap, ufo::ColorMap,
	//                ufo::VoidRegionMap>
	//     map_;

	ufo::AngularIntegrator<3> angular_integrator;
	// ufo::SimpleIntegrator<3>  simple_integrator;
	// ufo::InverseIntegrator<3> inverse_integrator;

	// TF
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
	std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
	std::string                                 map_frame_;
	double                                      timeout_;

	rclcpp::Publisher<ufo_interfaces::msg::Map>::SharedPtr map_pub_;

	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr insert_sub_;

	std::future<void> something_;
};
}  // namespace ufo_mapping

#endif  // UFO_MAPPING_SERVER_3D_HPP