// UFO
#include <ufo/cloud/ufo.hpp>
#include <ufo/math/math.hpp>
#include <ufo/math/numbers.hpp>

// UFO ROS
#include <ufo_mapping/server_3d.hpp>
#include <ufo_ros/ufo_ros.hpp>

// STL
#include <cstddef>

// ROS
#include <tf2/exceptions.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace ufo_mapping
{
MappingServer<3>::MappingServer(rclcpp::NodeOptions const& options)
    : rclcpp::Node("mapping_server_3d", options), map_(0.1)
{
	map_frame_ = this->declare_parameter<std::string>("map_frame", "map_gt");
	timeout_   = this->declare_parameter<double>("tf_timeout", 1.0);

	tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	map_pub_ = this->create_publisher<ufo_interfaces::msg::Map>("map", 10);

	insert_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
	    "/Evolo/Lidar/MidRes", 10,
	    [this](sensor_msgs::msg::PointCloud2::SharedPtr const msg) { insert(msg); });

	bool kitti = true;

	inverse_integrator.count_sample_method = ufo::CountSamplingMethod::MEAN;

	if (!kitti) {
		inverse_integrator.unordered_mode = false;
		inverse_integrator.nan_behavior   = ufo::InverseNaNBehavior::IGNORE;
		inverse_integrator.max_distance   = 130.0f;

		std::ifstream file;
		file.exceptions(std::ifstream::badbit);
		file.imbue(std::locale());
		file.open("/home/dduberg/Desktop/points.data", std::ios::in | std::ios::binary);

		std::uint64_t size;
		file.read(reinterpret_cast<char*>(&size), sizeof(size));
		std::vector<ufo::Vec3f> points(size);
		file.read(reinterpret_cast<char*>(points.data()), size * sizeof(ufo::Vec3f));

		// std::vector<ufo::OctCode> codes;
		// for (auto const& n : map_.query(ufo::pred::Z() < 0.01f, false, true)) {
		// 	codes.push_back(n.code);
		// }

		// for (auto const& c : codes) {
		// 	map_.voidRegionSet(c, true, true);
		// }

		for (auto& p : points) {
			if (-10.0f > p.z) {
				p.x = std::numeric_limits<float>::quiet_NaN();
				p.y = std::numeric_limits<float>::quiet_NaN();
				p.z = std::numeric_limits<float>::quiet_NaN();
			}
			if (0.0f == p.x && 0.0f == p.y && 0.0f == p.z) {
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
		inverse_integrator.unordered_mode = true;
		inverse_integrator.nan_behavior   = ufo::InverseNaNBehavior::IGNORE;
		inverse_integrator.max_distance   = 50.0f;

		double total_total_ms{};
		double total_read_cloud_ms{};
		double total_integrate_ms{};
		double total_publish_ms{};
		double total_propagate_ms{};

		std::filesystem::path dataset = R"(/home/dduberg/datasets/kitti/00)";
		std::ifstream         poses(dataset / "poses.txt", std::ios::in | std::ios::binary);
		std::ifstream calibration_fs(dataset / "calib.txt", std::ios::in | std::ios::binary);

		ufo::Transform3f calibration;
		{
			std::string s;
			while (std::getline(calibration_fs, s)) {
				std::stringstream        ss(s);
				std::vector<std::string> words;

				std::string tmp;
				while (getline(ss, tmp, ' ')) {
					words.push_back(tmp);
				}

				if ("Tr:" == words[0]) {
					calibration.rotation[0][0] = std::stof(words[1]);
					calibration.rotation[1][0] = std::stof(words[2]);
					calibration.rotation[2][0] = std::stof(words[3]);

					calibration.rotation[0][1] = std::stof(words[5]);
					calibration.rotation[1][1] = std::stof(words[6]);
					calibration.rotation[2][1] = std::stof(words[7]);

					calibration.rotation[0][2] = std::stof(words[9]);
					calibration.rotation[1][2] = std::stof(words[10]);
					calibration.rotation[2][2] = std::stof(words[11]);

					calibration.translation[0] = std::stof(words[4]);
					calibration.translation[1] = std::stof(words[8]);
					calibration.translation[2] = std::stof(words[12]);
					break;
				}
			}
		}

		auto calibration_inv = ufo::inverse(calibration);

		std::vector<std::filesystem::path> scans;
		for (auto const& dir_entry :
		     std::filesystem::directory_iterator{dataset / "velodyne"}) {
			if (dir_entry.is_regular_file()) {
				scans.push_back(dir_entry);
			}
		}

		std::sort(scans.begin(), scans.end());

		for (std::size_t iter{}; scans.size() > iter && rclcpp::ok(); ++iter) {
			auto const t1 = std::chrono::high_resolution_clock::now();

			// allocate 4 MB buffer (only ~130*4*4 KB are needed)
			std::vector<float> data(1'000'000);

			// pointers
			float* px = data.data() + 0;
			float* py = data.data() + 1;
			float* pz = data.data() + 2;
			// float* pr = data.data() + 3;

			// load point cloud
			std::FILE* stream;
			stream   = std::fopen(scans[iter].c_str(), "rb");
			auto num = std::fread(data.data(), sizeof(float), data.size(), stream) / 4;

			ufo::PointCloud<3, float> cloud;
			cloud.reserve(num);
			for (std::size_t i{}; num > i; ++i) {
				cloud.emplace_back(ufo::Vec3f{*px, *py, *pz});
				px += 4;
				py += 4;
				pz += 4;
				// pr += 4;
			}
			std::fclose(stream);

			ufo::Transform3f transform;
			{
				std::string s;
				std::getline(poses, s);
				std::stringstream        ss(s);
				std::vector<std::string> words;

				std::string tmp;
				while (getline(ss, tmp, ' ')) {
					words.push_back(tmp);
				}

				transform.rotation[0][0] = std::stof(words[0]);
				transform.rotation[1][0] = std::stof(words[1]);
				transform.rotation[2][0] = std::stof(words[2]);

				transform.rotation[0][1] = std::stof(words[4]);
				transform.rotation[1][1] = std::stof(words[5]);
				transform.rotation[2][1] = std::stof(words[6]);

				transform.rotation[0][2] = std::stof(words[8]);
				transform.rotation[1][2] = std::stof(words[9]);
				transform.rotation[2][2] = std::stof(words[10]);

				transform.translation[0] = std::stof(words[3]);
				transform.translation[1] = std::stof(words[7]);
				transform.translation[2] = std::stof(words[11]);

				transform = calibration_inv * (transform * calibration);
			}

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

		std::cout << "Average:\n";
		std::cout << "[Total: " << (total_total_ms / scans.size()) << " ms]";
		std::cout << "[Read cloud: " << (total_read_cloud_ms / scans.size()) << " ms]";
		std::cout << "[Integrate: " << (total_integrate_ms / scans.size()) << " ms]";
		std::cout << "[Publish: " << (total_publish_ms / scans.size()) << " ms]";
		std::cout << "[Propagate: " << (total_propagate_ms / scans.size()) << " ms]";
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
	// 	file.write(reinterpret_cast<char const*>(points.data()), size *
	// sizeof(ufo::Vec3f)); 	return;
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
		if (0.0f == p.x && 0.0f == p.y && 0.0f == p.z) {
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

	static auto viz_pub =
	    this->create_publisher<visualization_msgs::msg::MarkerArray>("viz", 10);
	if (0 < viz_pub->get_subscription_count()) {
		visualization_msgs::msg::MarkerArray markers;

		// Clear all existing markers
		visualization_msgs::msg::Marker clear_marker;
		clear_marker.header.frame_id = map_frame_;
		clear_marker.header.stamp    = msg->header.stamp;
		clear_marker.action          = visualization_msgs::msg::Marker::DELETEALL;
		markers.markers.push_back(clear_marker);

		std::vector<visualization_msgs::msg::Marker> void_region_marker(
		    map_.numDepthLevels());

		for (auto node : map_.query(ufo::pred::Leaf() && ufo::pred::VoidRegion() &&
		                            ufo::pred::Z() >= 0.01f)) {
			auto center = map_.center(node);
			auto depth  = map_.depth(node);

			geometry_msgs::msg::Point p;
			p.x = center.x;
			p.y = center.y;
			p.z = center.z;

			std_msgs::msg::ColorRGBA c;
			c.a = 1.0;
			c.b = 0.0;
			c.g = 1.0;
			c.r = 0.0;

			auto& vrm = void_region_marker[depth];
			vrm.points.push_back(p);
			vrm.colors.push_back(c);
		}

		for (std::size_t d{}; void_region_marker.size() > d; ++d) {
			auto& vrm = void_region_marker[d];
			if (vrm.points.empty()) {
				continue;
			}
			vrm.header             = clear_marker.header;
			vrm.id                 = d;
			vrm.ns                 = "void_region";
			vrm.type               = visualization_msgs::msg::Marker::CUBE_LIST;
			vrm.action             = visualization_msgs::msg::Marker::ADD;
			vrm.pose.orientation.w = 1.0;
			vrm.scale.x            = map_.length(d).x;
			vrm.scale.y            = map_.length(d).y;
			vrm.scale.z            = map_.length(d).z;
			vrm.color.a            = 1.0;
			// void_region_marker.lifetime           = 0.0;  // Forever
			vrm.frame_locked = false;
			markers.markers.push_back(vrm);
		}

		if (!markers.markers.empty()) {
			viz_pub->publish(markers);
		}
	}
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