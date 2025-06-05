// UFO ROS
#include <ufo_mapping/server_2d.hpp>

// STL
#include <cmath>

// ROS
#include <tf2/exceptions.hpp>

namespace ufo_mapping
{
MappingServer<2>::MappingServer(rclcpp::NodeOptions const& options)
    : rclcpp::Node("mapping_server_2d", options), map_(1.0)
{
	map_frame_ = this->declare_parameter<std::string>("map_frame", "map_gt");
	timeout_   = this->declare_parameter<double>("tf_timeout", 1.0);

	tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	map_pub_ = this->create_publisher<ufo_interfaces::msg::Map>("map", 10);

	insert_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
	    "insert",
	    rclcpp::SensorDataQoS().reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
	    [this](sensor_msgs::msg::LaserScan::SharedPtr const msg) { insert(msg); });

	// inverse_integrator.nan_behavior = ufo::InverseNaNBehavior::IGNORE;
	angular_integrator.max_distance = 130.0f;
}

void MappingServer<2>::insert(sensor_msgs::msg::LaserScan::SharedPtr const msg)
{
	static bool first = true;
	if (first) {
		first = false;

		std::vector<ufo::Vec2f> points;
		points.reserve(msg->ranges.size());

		float angle = msg->angle_min;
		for (float range : msg->ranges) {
			(void)range;  // Not needed since we want unit (direction) vectors
			points.emplace_back(std::cos(angle), std::sin(angle));
			angle += msg->angle_increment;
		}

		// inverse_integrator.generateConfig(map_, points);
	}

	auto transform = lookupTransform(map_frame_, msg->header.frame_id, msg->header.stamp,
	                                 rclcpp::Duration::from_seconds(timeout_));
	if (!transform.has_value()) {
		return;
	}

	auto const                t1 = std::chrono::high_resolution_clock::now();
	ufo::PointCloud<2, float> cloud;
	cloud.reserve(msg->ranges.size());

	float angle = msg->angle_min;
	for (float range : msg->ranges) {
		if (msg->range_min > range || msg->range_max < range || std::isnan(range)) {
			cloud.emplace_back(ufo::Vec2f(std::numeric_limits<float>::quiet_NaN(),
			                              std::numeric_limits<float>::quiet_NaN()));
		} else {
			cloud.emplace_back(ufo::Vec2f(range * std::cos(angle), range * std::sin(angle)));
		}
		angle += msg->angle_increment;
	}
	auto const t2 = std::chrono::high_resolution_clock::now();

	angular_integrator(ufo::execution::par, map_, cloud, transform.value());
	auto const t3 = std::chrono::high_resolution_clock::now();

	publishUpdate(msg->header.stamp);
	auto const t4 = std::chrono::high_resolution_clock::now();

	map_.propagate(ufo::execution::par);
	auto const t5 = std::chrono::high_resolution_clock::now();

	std::chrono::duration<double, std::milli> const total_ms     = t5 - t1;
	std::chrono::duration<double, std::milli> const ros_ufo_ms   = t2 - t1;
	std::chrono::duration<double, std::milli> const integrate_ms = t3 - t2;
	std::chrono::duration<double, std::milli> const publish_ms   = t4 - t3;
	std::chrono::duration<double, std::milli> const propagate_ms = t5 - t4;

	std::cout << "[Total: " << total_ms.count() << " ms]";
	std::cout << "[ROS->UFO: " << ros_ufo_ms.count() << " ms]";
	std::cout << "[Integrate: " << integrate_ms.count() << " ms]";
	std::cout << "[Publish: " << publish_ms.count() << " ms]";
	std::cout << "[Propagate: " << propagate_ms.count() << " ms]";
	std::cout << "[Num. nodes: " << map_.size() << "]";
	std::cout << '\n';

	ufo::PointCloud<3, float> dynamic_cloud;
	ufo::PointCloud<3, float> dynamic_secondary_cloud;
	ufo::PointCloud<3, float> static_cloud;

	// for (auto p : inverse_integrator.directions_) {
	// 	static_cloud.push_back(ufo::cast<float>(5.0 * p));
	// }

	for (auto const& p : cloud.view<0>()) {
		if (isnan(p)) {
			continue;
		}

		auto t_p = transform.value()(p);
		if (map_.voidRegion(t_p)) {
			dynamic_cloud.emplace_back(ufo::Vec3f(t_p, 0.0f));
			// } else if (map_.voidRegionSecondary(t_p)) {
			// 	dynamic_secondary_cloud.push_back(ufo::Vec3f(t_p, 0.0f));
		} else {
			static_cloud.emplace_back(ufo::Vec3f(t_p, 0.0f));
		}
	}

	auto dynamic_msg           = ufo_ros::toMsg(dynamic_cloud);
	auto dynamic_secondary_msg = ufo_ros::toMsg(dynamic_secondary_cloud);
	auto static_msg            = ufo_ros::toMsg(static_cloud);

	dynamic_msg.header.frame_id = map_frame_;
	dynamic_msg.header.stamp    = msg->header.stamp;
	// dynamic_msg.header           = msg->header;
	dynamic_secondary_msg.header = dynamic_msg.header;
	static_msg.header            = dynamic_msg.header;

	static auto dynamic_pub =
	    this->create_publisher<sensor_msgs::msg::PointCloud2>("dynamic", 10);
	static auto dynamic_secondary_pub =
	    this->create_publisher<sensor_msgs::msg::PointCloud2>("dynamic_secondary", 10);
	static auto static_pub =
	    this->create_publisher<sensor_msgs::msg::PointCloud2>("static", 10);

	dynamic_pub->publish(dynamic_msg);
	dynamic_secondary_pub->publish(dynamic_secondary_msg);
	static_pub->publish(static_msg);
}

std::optional<ufo::Transform2f> MappingServer<2>::lookupTransform(
    std::string const& target_frame, std::string const& source_frame,
    rclcpp::Time const& time, rclcpp::Duration const& timeout) const
{
	try {
		auto t = tf_buffer_->lookupTransform(target_frame, source_frame, time, timeout);
		ufo::Transform3f tf = ufo_ros::fromMsg(t.transform);
		return ufo::Transform2f(ufo::roll(ufo::Quat(tf.rotation)),
		                        ufo::Vec2f(tf.translation));
	} catch (tf2::TransformException const& ex) {
		RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
		            source_frame.c_str(), target_frame.c_str(), ex.what());
		return {};
	}
}

void MappingServer<2>::publishUpdate(rclcpp::Time const& time) const
{
	if (0 < map_pub_->get_subscription_count()) {
		auto out_msg = ufo_ros::toMsg(map_, ufo::pred::Leaf() && ufo::pred::Modified());
		out_msg.header.frame_id = map_frame_;
		out_msg.header.stamp    = time;
		map_pub_->publish(out_msg);
	}
}
}  // namespace ufo_mapping
