// UFO
#include <ufo/cloud/ufo.hpp>
#include <ufo/math/math.hpp>
#include <ufo/math/numbers.hpp>

// UFO ROS
#include <ufo_mapping/server_3d.hpp>
#include <ufo_ros/ufo_ros.hpp>

// STL
#include <cstddef>
#include <functional>

// ROS
#include <tf2/exceptions.hpp>

namespace ufo_mapping
{
MappingServer<3>::MappingServer(rclcpp::NodeOptions const& options)
    : rclcpp::Node("mapping_server_3d", options), map_(1.0)
{
	map_frame_ = this->declare_parameter<std::string>("map_frame", "map_gt");
	timeout_   = this->declare_parameter<double>("tf_timeout", 1.0);

	tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	map_pub_ = this->create_publisher<ufo_interfaces::msg::Map>("map", 10);

	insert_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
	    "insert", 10,
	    [this](sensor_msgs::msg::PointCloud2::SharedPtr const msg) { insert(msg); });

	bool kitti = false;

	if (!kitti) {
		inverse_integrator.treat_nan_as_infinity = true;
		inverse_integrator.max_distance          = 130.0f;

		std::ifstream file;
		file.exceptions(std::ifstream::badbit);
		file.imbue(std::locale());
		file.open("/home/dduberg/Desktop/points.data", std::ios::in | std::ios::binary);

		std::uint64_t size;
		file.read(reinterpret_cast<char*>(&size), sizeof(size));
		std::vector<ufo::Vec3f> points(size);
		file.read(reinterpret_cast<char*>(points.data()), size * sizeof(ufo::Vec3f));

		std::vector<ufo::OctCode> codes;
		for (auto const& n : map_.query(ufo::pred::Z() < 0.01f, false, true)) {
			codes.push_back(n.code);
		}

		for (auto const& c : codes) {
			map_.voidRegionSet(c, true, true);
		}

		for (auto& p : points) {
			if (-10.0f > p.z) {
				p.x = std::numeric_limits<float>::quiet_NaN();
				p.y = std::numeric_limits<float>::quiet_NaN();
				p.z = std::numeric_limits<float>::quiet_NaN();
			}
		}

		inverse_integrator.generateConfig(map_, points);
	}

	// inverse_integrator.generateConfig(map_, 0.0, 2.0 * ufo::numbers::pi_v<double>,
	//                                   2.0 * ufo::numbers::pi_v<double> / 3600.0,
	//                                   ufo::radians(-16.0), ufo::radians(16.0),
	//                                   ufo::radians(1.0),
	//                                   ufo::ScanOrder::HORIZONTAL_MAJOR);

	if (kitti) {
		inverse_integrator.treat_nan_as_infinity = false;
		inverse_integrator.max_distance          = 50.0f;

		double total_total_ms{};
		double total_read_cloud_ms{};
		double total_integrate_ms{};
		double total_publish_ms{};
		double total_propagate_ms{};

		std::size_t num_clouds = 50;
		for (std::size_t iter{}; num_clouds > iter && rclcpp::ok(); ++iter) {
			auto const t1 = std::chrono::high_resolution_clock::now();

			ufo::PointCloud<3, float, ufo::Color> cloud;
			ufo::Transform3f                      transform;
			ufo::readCloudUFO("/home/dduberg/ufo3/lib/viz/resources/kitti/ufo/0000_cloud.ufo",
			                  cloud, transform);
			transform.translation.z -= iter * 10;

			if (0 == iter) {
				inverse_integrator.generateConfig(map_, cloud);
			}

			auto const t2 = std::chrono::high_resolution_clock::now();

			inverse_integrator(ufo::execution::par, map_, cloud, transform, false);
			auto const t3 = std::chrono::high_resolution_clock::now();

			// publishUpdate(msg->header.stamp);
			auto const t4 = std::chrono::high_resolution_clock::now();

			// map_.modifiedPropagate(ufo::execution::par, ufo::MapType::ALL, true, false);
			auto const t5 = std::chrono::high_resolution_clock::now();

			std::chrono::duration<double, std::milli> const total_ms      = t5 - t1;
			std::chrono::duration<double, std::milli> const read_cloud_ms = t2 - t1;
			std::chrono::duration<double, std::milli> const integrate_ms  = t3 - t2;
			std::chrono::duration<double, std::milli> const publish_ms    = t4 - t3;
			std::chrono::duration<double, std::milli> const propagate_ms  = t5 - t4;

			if (0 != iter) {
				total_total_ms += total_ms.count();
				total_read_cloud_ms += read_cloud_ms.count();
				total_integrate_ms += integrate_ms.count();
				total_publish_ms += publish_ms.count();
				total_propagate_ms += propagate_ms.count();
			}

			// std::cout << "[Total: " << total_ms.count() << " ms]";
			// std::cout << "[Read cloud: " << read_cloud_ms.count() << " ms]";
			// std::cout << "[Integrate: " << integrate_ms.count() << " ms]";
			// std::cout << "[Publish: " << publish_ms.count() << " ms]";
			// std::cout << "[Propagate: " << propagate_ms.count() << " ms]";
			// std::cout << "[Num. nodes: " << map_.size() << "]";
			// std::cout << '\n';
		}

		std::cout << "Total:\n";
		std::cout << "[Total: " << total_total_ms << " ms]";
		std::cout << "[Read cloud: " << total_read_cloud_ms << " ms]";
		std::cout << "[Integrate: " << total_integrate_ms << " ms]";
		std::cout << "[Publish: " << total_publish_ms << " ms]";
		std::cout << "[Propagate: " << total_propagate_ms << " ms]";
		std::cout << "[Num. nodes: " << map_.size() << "]";
		std::cout << '\n';

		--num_clouds;

		std::cout << "Average:\n";
		std::cout << "[Total: " << (total_total_ms / num_clouds) << " ms]";
		std::cout << "[Read cloud: " << (total_read_cloud_ms / num_clouds) << " ms]";
		std::cout << "[Integrate: " << (total_integrate_ms / num_clouds) << " ms]";
		std::cout << "[Publish: " << (total_publish_ms / num_clouds) << " ms]";
		std::cout << "[Propagate: " << (total_propagate_ms / num_clouds) << " ms]";
		std::cout << "[Num. nodes: " << map_.size() << "]";
		std::cout << '\n';
	}
}

void MappingServer<3>::insert(sensor_msgs::msg::PointCloud2::SharedPtr const msg)
{
	// {
	// 	auto cloud = ufo_ros::fromMsg(*msg);

	// 	static std::vector<ufo::Vec3f> points(
	// 	    cloud.size(), ufo::Vec3f(std::numeric_limits<float>::quiet_NaN()));
	// 	for (std::size_t i{}; cloud.size() > i; ++i) {
	// 		auto p = ufo::get<0>(cloud)[i];
	// 		if (!isnan(p) && ufo::Vec3f{} != p) {
	// 			points[i] = p;
	// 		}
	// 	}

	// 	std::ofstream file;
	// 	file.exceptions(std::ifstream::badbit);
	// 	file.imbue(std::locale());
	// 	file.open("/home/dduberg/Desktop/points.data", std::ios::out | std::ios::binary);

	// 	std::uint64_t size = points.size();
	// 	file.write(reinterpret_cast<char*>(&size), sizeof(size));
	// 	file.write(reinterpret_cast<char const*>(points.data()), size * sizeof(ufo::Vec3f));
	// 	return;
	// }

	auto transform = lookupTransform(map_frame_, msg->header.frame_id, msg->header.stamp,
	                                 rclcpp::Duration::from_seconds(timeout_));
	if (!transform.has_value()) {
		return;
	}

	auto const t1    = std::chrono::high_resolution_clock::now();
	auto       cloud = ufo_ros::fromMsg(*msg);
	auto const t2    = std::chrono::high_resolution_clock::now();

	// TODO: Remove this
	for (auto& p : ufo::get<0>(cloud)) {
		auto t = transform.value()(p);
		if (0.0f > t.z) {
			p.x = std::numeric_limits<float>::quiet_NaN();
			p.y = std::numeric_limits<float>::quiet_NaN();
			p.z = std::numeric_limits<float>::quiet_NaN();
		}
	}

	inverse_integrator(ufo::execution::par, map_, cloud, transform.value(), false);
	auto const t3 = std::chrono::high_resolution_clock::now();

	publishUpdate(msg->header.stamp);
	auto const t4 = std::chrono::high_resolution_clock::now();

	map_.modifiedPropagate(ufo::execution::par, false);
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

	for (auto const& p : ufo::get<0>(cloud)) {
		if (isnan(p)) {
			continue;
		}

		auto t_p = transform.value()(p);
		if (map_.voidRegion(t_p)) {
			dynamic_cloud.push_back(t_p);
		} else if (map_.voidRegionSecondary(t_p)) {
			dynamic_secondary_cloud.push_back(t_p);
		} else {
			static_cloud.push_back(t_p);
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

std::optional<ufo::Transform3f> MappingServer<3>::lookupTransform(
    std::string const& target_frame, std::string const& source_frame,
    rclcpp::Time const& time, rclcpp::Duration const& timeout) const
{
	try {
		auto t = tf_buffer_->lookupTransform(target_frame, source_frame, time, timeout);
		return ufo_ros::fromMsg(t.transform);
	} catch (tf2::TransformException const& ex) {
		RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
		            source_frame.c_str(), target_frame.c_str(), ex.what());
		return {};
	}
}

void MappingServer<3>::publishUpdate(rclcpp::Time const& time) const
{
	if (0 < map_pub_->get_subscription_count()) {
		// auto out_msg = ufo_ros::toMsg(map_);
		auto out_msg = ufo_ros::toMsg(map_, ufo::pred::Leaf() && ufo::pred::Modified());
		// auto out_msg = ufo_ros::toMsg(map_, ufo::pred::Modified() && ufo::pred::Leaf());
		// auto out_msg = ufo_ros::toMsg(map_, ufo::pred::Modified());
		// auto out_msg = ufo_ros::toMsg(
		//     map_, ufo::pred::Modified() && ufo::pred::Leaf() && ufo::pred::PureLeaf());

		// ufo::Map3D<ufo::OccupancyMap, ufo::ColorMap, ufo::VoidRegionMap> map2;
		// std::cout << "Before:\n";
		// std::cout << map_.occupancy(ufo::Vec3f(0, 0, 0)) << '\n';
		// std::cout << map2.occupancy(ufo::Vec3f(0, 0, 0)) << '\n';

		// RCLCPP_INFO(this->get_logger(), "Reading map");
		// ufo_ros::fromMsg(out_msg, map2);
		// RCLCPP_INFO(this->get_logger(), "Read map");

		// std::cout << "After:\n";
		// std::cout << map_.occupancy(ufo::Vec3f(0, 0, 0)) << '\n';
		// std::cout << map2.occupancy(ufo::Vec3f(0, 0, 0)) << '\n';

		// RCLCPP_INFO(this->get_logger(), "Comparing maps");
		// if (map2 != map_) {
		// 	RCLCPP_ERROR(this->get_logger(), "Serialization error");
		// }
		// RCLCPP_INFO(this->get_logger(), "Compared maps");

		out_msg.header.frame_id = map_frame_;
		out_msg.header.stamp    = time;
		map_pub_->publish(out_msg);
	}
}
}  // namespace ufo_mapping