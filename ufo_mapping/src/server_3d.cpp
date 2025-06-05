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
    : rclcpp::Node("mapping_server_3d", options), map_(0.01)
{
	map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
	timeout_   = this->declare_parameter<double>("tf_timeout", 1.0);

	tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	// ufo::Map3D<ufo::OccupancyMap> map_1;
	// ufo::Map3D<ufo::ColorMap>     map_2 = map_1;

	map_pub_ = this->create_publisher<ufo_interfaces::msg::Map>("map", 10);

	insert_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
	    "insert", 10,
	    [this](sensor_msgs::msg::PointCloud2::SharedPtr const msg) { insert(msg); });

	angular_integrator.max_distance = 2.0f;

	// std::filesystem::path dataset = R"(/home/dduberg/datasets/kitti/00)";
	// std::ifstream         poses(dataset / "poses.txt", std::ios::in | std::ios::binary);
	// std::ifstream calibration_fs(dataset / "calib.txt", std::ios::in | std::ios::binary);

	// ufo::Transform3f calibration;
	// {
	// 	std::string s;
	// 	while (std::getline(calibration_fs, s)) {
	// 		std::stringstream        ss(s);
	// 		std::vector<std::string> words;

	// 		std::string tmp;
	// 		while (getline(ss, tmp, ' ')) {
	// 			words.push_back(tmp);
	// 		}

	// 		if ("Tr:" == words[0]) {
	// 			calibration.rotation[0][0] = std::stof(words[1]);
	// 			calibration.rotation[1][0] = std::stof(words[2]);
	// 			calibration.rotation[2][0] = std::stof(words[3]);

	// 			calibration.rotation[0][1] = std::stof(words[5]);
	// 			calibration.rotation[1][1] = std::stof(words[6]);
	// 			calibration.rotation[2][1] = std::stof(words[7]);

	// 			calibration.rotation[0][2] = std::stof(words[9]);
	// 			calibration.rotation[1][2] = std::stof(words[10]);
	// 			calibration.rotation[2][2] = std::stof(words[11]);

	// 			calibration.translation[0] = std::stof(words[4]);
	// 			calibration.translation[1] = std::stof(words[8]);
	// 			calibration.translation[2] = std::stof(words[12]);
	// 			break;
	// 		}
	// 	}
	// }

	// auto calibration_inv = ufo::inverse(calibration);

	// std::vector<std::filesystem::path> scans;
	// for (auto const& dir_entry :
	//      std::filesystem::directory_iterator{dataset / "velodyne"}) {
	// 	if (dir_entry.is_regular_file()) {
	// 		scans.push_back(dir_entry);
	// 	}
	// }

	// std::sort(scans.begin(), scans.end());

	// for (std::size_t iter{}; scans.size() > iter && rclcpp::ok(); ++iter) {
	// 	auto const t1 = std::chrono::high_resolution_clock::now();

	// 	// allocate 4 MB buffer (only ~130*4*4 KB are needed)
	// 	std::vector<float> data(1'000'000);

	// 	// pointers
	// 	float* px = data.data() + 0;
	// 	float* py = data.data() + 1;
	// 	float* pz = data.data() + 2;
	// 	// float* pr = data.data() + 3;

	// 	// load point cloud
	// 	std::FILE* stream;
	// 	stream   = std::fopen(scans[iter].c_str(), "rb");
	// 	auto num = std::fread(data.data(), sizeof(float), data.size(), stream) / 4;

	// 	ufo::PointCloud<3, float, ufo::Color> cloud;
	// 	cloud.reserve(num);
	// 	for (std::size_t i{}; num > i; ++i) {
	// 		cloud.emplace_back(ufo::Vec3f{*px, *py, *pz});
	// 		px += 4;
	// 		py += 4;
	// 		pz += 4;
	// 		// pr += 4;
	// 	}
	// 	std::fclose(stream);

	// 	ufo::Transform3f transform;
	// 	{
	// 		std::string s;
	// 		std::getline(poses, s);
	// 		std::stringstream        ss(s);
	// 		std::vector<std::string> words;

	// 		std::string tmp;
	// 		while (getline(ss, tmp, ' ')) {
	// 			words.push_back(tmp);
	// 		}

	// 		transform.rotation[0][0] = std::stof(words[0]);
	// 		transform.rotation[1][0] = std::stof(words[1]);
	// 		transform.rotation[2][0] = std::stof(words[2]);

	// 		transform.rotation[0][1] = std::stof(words[4]);
	// 		transform.rotation[1][1] = std::stof(words[5]);
	// 		transform.rotation[2][1] = std::stof(words[6]);

	// 		transform.rotation[0][2] = std::stof(words[8]);
	// 		transform.rotation[1][2] = std::stof(words[9]);
	// 		transform.rotation[2][2] = std::stof(words[10]);

	// 		transform.translation[0] = std::stof(words[3]);
	// 		transform.translation[1] = std::stof(words[7]);
	// 		transform.translation[2] = std::stof(words[11]);

	// 		transform = calibration_inv * (transform * calibration);
	// 	}

	// 	insert(cloud, transform, this->now());
	// }
}

void MappingServer<3>::insert(sensor_msgs::msg::PointCloud2::SharedPtr const msg)
{
	auto const transform =
	    lookupTransform(map_frame_, msg->header.frame_id, msg->header.stamp,
	                    rclcpp::Duration::from_seconds(timeout_));
	if (!transform.has_value()) {
		return;
	}

	std::cerr << "Before fromMsg\n";
	auto const cloud = ufo_ros::fromMsg<3, float, ufo::Color>(*msg);
	std::cerr << "After fromMsg\n";

	insert(cloud, transform.value(), rclcpp::Time(msg->header.stamp));
}

void MappingServer<3>::insert(ufo::PointCloud<3, float, ufo::Color> const& cloud,
                              ufo::Transform<3, float> const&              transform,
                              rclcpp::Time const&                          time)
{
	auto const t1 = std::chrono::high_resolution_clock::now();
	angular_integrator(ufo::execution::par, map_, cloud, transform, ufo::Vec<3, float>{},
	                   something_);

	auto const t2 = std::chrono::high_resolution_clock::now();

	something_ = std::async(std::launch::async, [this, cloud, transform, time, t1, t2]() {
		map_.propagate(ufo::execution::par);

		auto const t3 = std::chrono::high_resolution_clock::now();

		publishUpdate(time);

		auto const t4 = std::chrono::high_resolution_clock::now();

		map_.modifiedReset();

		auto const t5 = std::chrono::high_resolution_clock::now();

		std::chrono::duration<double, std::milli> const integrate_ms      = t2 - t1;
		std::chrono::duration<double, std::milli> const propagate_ms      = t3 - t2;
		std::chrono::duration<double, std::milli> const publish_ms        = t4 - t3;
		std::chrono::duration<double, std::milli> const reset_modified_ms = t5 - t4;
		std::chrono::duration<double, std::milli> const total_ms          = t5 - t1;

		static std::size_t iteration{};

		std::cout << "[Total: " << total_ms.count() << " ms]";
		std::cout << "[Integrate: " << integrate_ms.count() << " ms]";
		std::cout << "[Propagate: " << propagate_ms.count() << " ms]";
		std::cout << "[Publish: " << publish_ms.count() << " ms]";
		std::cout << "[Reset modified: " << reset_modified_ms.count() << " ms]";
		std::cout << "[Num. nodes: " << map_.size() << "]";
		std::cout << "[Iter.: " << (++iteration) << "]";
		std::cout << '\n';

		ufo::PointCloud<3, float> dynamic_cloud;
		ufo::PointCloud<3, float> static_cloud;

		for (auto const& p : cloud.view<0>()) {
			if (isnan(p)) {
				continue;
			}

			auto t_p = transform(p);
			if (map_.voidRegion(t_p)) {
				dynamic_cloud.emplace_back(t_p);
			} else {
				static_cloud.emplace_back(t_p);
			}
		}

		auto dynamic_msg = ufo_ros::toMsg(dynamic_cloud);
		auto static_msg  = ufo_ros::toMsg(static_cloud);

		dynamic_msg.header.frame_id = map_frame_;
		dynamic_msg.header.stamp    = time;
		static_msg.header           = dynamic_msg.header;

		static auto dynamic_pub =
		    this->create_publisher<sensor_msgs::msg::PointCloud2>("dynamic", 10);
		static auto static_pub =
		    this->create_publisher<sensor_msgs::msg::PointCloud2>("static", 10);

		dynamic_pub->publish(dynamic_msg);
		static_pub->publish(static_msg);
	});

	something_.wait();
}

std::optional<ufo::Transform3f> MappingServer<3>::lookupTransform(
    std::string const& target_frame, std::string const& source_frame,
    rclcpp::Time const& time, rclcpp::Duration const& timeout) const
{
	try {
		if ('/' == source_frame[0]) {
			auto t = tf_buffer_->lookupTransform(target_frame, source_frame.substr(1), time,
			                                     timeout);
			return ufo_ros::fromMsg(t.transform);
		} else {
			auto t = tf_buffer_->lookupTransform(target_frame, source_frame, time, timeout);
			return ufo_ros::fromMsg(t.transform);
		}
	} catch (tf2::TransformException const& ex) {
		RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
		            source_frame.c_str(), target_frame.c_str(), ex.what());
		return {};
	}
}

void MappingServer<3>::publishUpdate(rclcpp::Time const& time) const
{
	if (0 < (map_pub_->get_subscription_count() +
	         map_pub_->get_intra_process_subscription_count())) {
		auto out_msg = ufo_ros::toMsg(map_, ufo::pred::Leaf() && ufo::pred::Modified());
		out_msg.header.frame_id = map_frame_;
		out_msg.header.stamp    = time;
		map_pub_->publish(out_msg);
	}
}
}  // namespace ufo_mapping