#ifndef UFO_ROS_UFO_ROS_HPP
#define UFO_ROS_UFO_ROS_HPP

// UFO
#include <ufo/cloud/point_cloud.hpp>
#include <ufo/map/header.hpp>
#include <ufo/map/type.hpp>
#include <ufo/map/ufomap.hpp>
#include <ufo/math/quat.hpp>
#include <ufo/math/transform.hpp>
#include <ufo/utility/enum.hpp>
#include <ufo/vision/color.hpp>

// UFO ROS
#include <ufo_interfaces/msg/map.hpp>
#include <ufo_interfaces/msg/map_info.hpp>
#include <ufo_interfaces/msg/map_type_info.hpp>
#include <ufo_interfaces/msg/nav_map.hpp>

// STL
#include <cstddef>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/point_field_conversion.hpp>

namespace ufo_ros
{
/**************************************************************************************
|                                                                                     |
|                                      From msg                                       |
|                                                                                     |
**************************************************************************************/

[[nodiscard]] inline ufo::MapTypeInfo fromMsg(ufo_interfaces::msg::MapTypeInfo const& msg)
{
	ufo::MapTypeInfo res;
	res.type = static_cast<ufo::MapType>(msg.id);
	res.size = msg.size;
	return res;
}

[[nodiscard]] inline ufo::MapHeader fromMsg(ufo_interfaces::msg::MapInfo const& msg)
{
	ufo::MapHeader res;
	res.major = msg.version.major;
	res.minor = msg.version.minor;
	res.patch = msg.version.patch;
	res.leaf_node_length.assign(msg.leaf_node_length.begin(), msg.leaf_node_length.end());
	res.num_depth_levels = msg.num_depth_levels;
	res.num_blocks       = msg.num_blocks;
	res.num_nodes        = msg.num_nodes;
	res.map_info.resize(msg.map.size());
	for (std::size_t i{}; msg.map.size() > i; ++i) {
		res.map_info[i] = fromMsg(msg.map[i]);
	}
	return res;
}

template <class Map>
void fromMsg(ufo_interfaces::msg::Map const& msg, Map& map,
             ufo::MapType map_types = ufo::MapType::ALL, bool propagate = true)
{
	if (msg.data.empty()) {
		return;
	}

	ufo::MapHeader header = fromMsg(msg.info);

	ufo::Buffer buffer;
	buffer.write(msg.data.data(), msg.data.size());
	map.readData(buffer, header, map_types, propagate);
}

template <std::size_t Dim>
void fromMsg(ufo_interfaces::msg::NavMap const& msg)
{
	// TODO: Implement
}

/*!
 * @brief Convert a Point message to its equivalent ufo representation.
 *
 * @param msg A Point message.
 * @return The converted Point message as a ufo Vec3.
 */
template <class T = float>
[[nodiscard]] ufo::Vec3<T> fromMsg(geometry_msgs::msg::Point const& msg)
{
	return ufo::Vec3<T>(static_cast<T>(msg.x), static_cast<T>(msg.y),
	                    static_cast<T>(msg.z));
}

/*!
 * @brief Convert a Vector3 message to its equivalent ufo representation.
 *
 * @param msg A Vector3 message.
 * @return The converted Vector3 message as a ufo Vec3.
 */
template <class T = float>
[[nodiscard]] ufo::Vec3<T> fromMsg(geometry_msgs::msg::Vector3 const& msg)
{
	return ufo::Vec3<T>(static_cast<T>(msg.x), static_cast<T>(msg.y),
	                    static_cast<T>(msg.z));
}

/*!
 * @brief Convert a Quaternion message to its equivalent ufo representation.
 *
 * @param msg A Quaternion message.
 * @return The converted Quaternion message as a ufo Quat.
 */
template <class T = float>
[[nodiscard]] ufo::Quat<T> fromMsg(geometry_msgs::msg::Quaternion const& msg)
{
	return ufo::Quat<T>(static_cast<T>(msg.w), static_cast<T>(msg.x), static_cast<T>(msg.y),
	                    static_cast<T>(msg.z));
}

/*!
 * @brief Convert a Transform message to its equivalent ufo representation.
 *
 * @param msg A Transform message.
 * @return The converted Transform message as a ufo Transform3.
 */
template <class T = float>
[[nodiscard]] ufo::Transform3<T> fromMsg(geometry_msgs::msg::Transform const& msg)
{
	return ufo::Transform3<T>(fromMsg<T>(msg.rotation), fromMsg<T>(msg.translation));
}

/*!
 * @brief Convert a Pose message to its equivalent ufo representation.
 *
 * @param msg A Pose message.
 * @return The converted Pose message as a ufo Transform3.
 */
template <class T = float>
[[nodiscard]] ufo::Transform3<T> fromMsg(geometry_msgs::msg::Pose const& msg)
{
	return ufo::Transform3<T>(fromMsg<T>(msg.orientation), fromMsg<T>(msg.position));
}

inline bool hasField(sensor_msgs::msg::PointCloud2 const& msg,
                     std::string const&                   field_name)
{
	for (auto const& field : msg.fields) {
		// std::cout << "field: " << field.name << '\n';
		if (field.name == field_name) {
			return true;
		}
	}
	return false;
}

template <std::size_t Dim, class T, class... Rest>
void fromMsg(sensor_msgs::msg::PointCloud2 const& msg,
             ufo::PointCloud<Dim, T, Rest...>&    out)
{
	out.resize(msg.height * msg.width);

	if (out.empty()) {
		return;
	}

	sensor_msgs::PointCloud2ConstIterator<T> iter_x(msg, "x");
	sensor_msgs::PointCloud2ConstIterator<T> iter_y(msg, "y");
	sensor_msgs::PointCloud2ConstIterator<T> iter_z(msg, "z");
	for (auto& p : ufo::get<0>(out)) {
		p.x = *iter_x;
		++iter_x;
		p.y = *iter_y;
		++iter_y;
		if constexpr (3 <= Dim) {
			p.z = *iter_z;
			++iter_z;
		}
	}

	if constexpr (ufo::contains_type_v<ufo::Color, Rest...>) {
		// TODO: Add check if color exists and if alpha exists

		bool has_r = hasField(msg, "r");
		bool has_g = hasField(msg, "g");
		bool has_b = hasField(msg, "b");

		if (hasField(msg, "rgba") || (has_r && has_g && has_b && hasField(msg, "a"))) {
			sensor_msgs::PointCloud2ConstIterator<ufo::Color::value_type> iter_r(msg, "r");
			sensor_msgs::PointCloud2ConstIterator<ufo::Color::value_type> iter_g(msg, "g");
			sensor_msgs::PointCloud2ConstIterator<ufo::Color::value_type> iter_b(msg, "b");
			sensor_msgs::PointCloud2ConstIterator<ufo::Color::value_type> iter_a(msg, "a");
			for (auto& c : ufo::get<ufo::Color>(out)) {
				c.red   = *iter_r;
				c.green = *iter_g;
				c.blue  = *iter_b;
				c.alpha = *iter_a;
				++iter_r;
				++iter_g;
				++iter_b;
				++iter_a;
			}
		} else if (hasField(msg, "rgb") || (has_r && has_g && has_b)) {
			sensor_msgs::PointCloud2ConstIterator<ufo::Color::value_type> iter_r(msg, "r");
			sensor_msgs::PointCloud2ConstIterator<ufo::Color::value_type> iter_g(msg, "g");
			sensor_msgs::PointCloud2ConstIterator<ufo::Color::value_type> iter_b(msg, "b");
			for (auto& c : ufo::get<ufo::Color>(out)) {
				c.red   = *iter_r;
				c.green = *iter_g;
				c.blue  = *iter_b;
				c.alpha = 255;  // Default alpha value
				++iter_r;
				++iter_g;
				++iter_b;
			}
		}
	}

	// TODO: Implement

	if constexpr (ufo::contains_type_v<ufo::Label, Rest...>) {
		if (hasField(msg, "l")) {
			sensor_msgs::PointCloud2ConstIterator<ufo::label_t> iter_l(msg, "l");
			for (auto& l : ufo::get<ufo::Label>(out)) {
				l = *iter_l;
				++iter_l;
			}
		} else if (hasField(msg, "label")) {
			sensor_msgs::PointCloud2ConstIterator<ufo::label_t> iter_l(msg, "label");
			for (auto& l : ufo::get<ufo::Label>(out)) {
				l = *iter_l;
				++iter_l;
			}
		}
	}
}

template <std::size_t Dim, class T = float, class... Ts>
[[nodiscard]] ufo::PointCloud<Dim, T, Ts...> fromMsg(
    sensor_msgs::msg::PointCloud2 const& msg)
{
	ufo::PointCloud<Dim, T, Ts...> cloud;
	fromMsg(msg, cloud);
	return cloud;
}

template <class... Ts>
[[nodiscard]] ufo::PointCloud<3, float, Ts...> fromMsg(
    sensor_msgs::msg::PointCloud2 const& msg)
{
	return fromMsg<3, float, Ts...>(msg);
}

// [[nodiscard]] inline ufo::PointCloud<2, float> fromMsg(
//     sensor_msgs::msg::LaserScan const& msg, bool out_of_range_to_nan)
// {
// TODO: Implement
// ufo::PointCloud<2, float> res;
// res.reserve(msg->ranges.size());

// float angle = msg->angle_min;
// for (float range : msg->ranges) {
// 	if (msg->range_min > range || msg->range_max < range || std::isnan(range)) {
// 		res.push_back(out_of_range_to_nan ? std::numeric_limits<float>::quiet_NaN() : )
// 	}
// 	// TODO: Implement
// }

// return res;
// }

/**************************************************************************************
|                                                                                     |
|                                       To msg                                        |
|                                                                                     |
**************************************************************************************/

[[nodiscard]] inline ufo_interfaces::msg::MapTypeInfo toMsg(ufo::MapTypeInfo const& a)
{
	ufo_interfaces::msg::MapTypeInfo res;
	res.type = ufo::to_string(a.type);
	res.id   = ufo::to_underlying(a.type);
	res.size = a.size;
	return res;
}

[[nodiscard]] inline ufo_interfaces::msg::MapInfo toMsg(ufo::MapHeader const& a)
{
	ufo_interfaces::msg::MapInfo res;
	res.version.major = a.major;
	res.version.minor = a.minor;
	res.version.patch = a.patch;
	res.dimensions    = a.leaf_node_length.size();
	res.leaf_node_length.assign(a.leaf_node_length.begin(), a.leaf_node_length.end());
	res.num_depth_levels = a.num_depth_levels;
	res.num_blocks       = a.num_blocks;
	res.num_nodes        = a.num_nodes;
	res.compressed       = false;
	res.map.resize(a.map_info.size());
	for (std::size_t i{}; a.map_info.size() > i; ++i) {
		res.map[i] = toMsg(a.map_info[i]);
	}
	res.map[0].type = "tree";
	return res;
}

template <class Map, class Predicate = ufo::pred::Leaf,
          std::enable_if_t<ufo::pred::is_pred_v<Predicate, Map>, bool> = true>
[[nodiscard]] ufo_interfaces::msg::Map toMsg(Map const&       a,
                                             Predicate const& pred  = ufo::pred::Leaf{},
                                             ufo::MapType map_types = ufo::MapType::ALL)
{
	ufo::Buffer buffer;
	auto        map_header = a.writeData(buffer, pred, map_types);

	ufo_interfaces::msg::Map res;
	res.info = toMsg(map_header);
	// TODO: Make sure size is correct
	res.data.resize(buffer.size());
	buffer.read(res.data.data(), res.data.size());

	return res;
}

template <std::size_t Dim>
[[nodiscard]] ufo_interfaces::msg::NavMap toMsg()
{
	ufo_interfaces::msg::NavMap res;

	// TODO: Implement

	return res;
}

// TODO: Do something with point?

template <class T>
[[nodiscard]] geometry_msgs::msg::Vector3 toMsg(ufo::Vec3<T> const& a)
{
	geometry_msgs::msg::Vector3 res;
	res.x = a.x;
	res.y = a.y;
	res.z = a.z;
	return res;
}

template <class T>
[[nodiscard]] geometry_msgs::msg::Quaternion toMsg(ufo::Quat<T> const& a)
{
	geometry_msgs::msg::Quaternion res;
	res.w = a.w;
	res.x = a.x;
	res.y = a.y;
	res.z = a.z;
	return res;
}

template <class T>
[[nodiscard]] geometry_msgs::msg::Transform toMsg(ufo::Transform3<T> const& a)
{
	geometry_msgs::msg::Transform res;
	res.rotation    = toMsg(a.rotation);
	res.translation = toMsg(a.translation);
	return res;
}

// TODO: Do something with pose?

template <std::size_t Dim, class T, class... Rest>
[[nodiscard]] sensor_msgs::msg::PointCloud2 toMsg(
    ufo::PointCloud<Dim, T, Rest...> const& a)
{
	sensor_msgs::msg::PointCloud2 res;
	res.height = 1;
	res.width  = a.size();

	sensor_msgs::PointCloud2Modifier modifier(res);

	if constexpr (ufo::contains_type_v<ufo::Color, Rest...>) {
		modifier.setPointCloud2FieldsByString(2, "xyz", "rgba");
	} else {
		modifier.setPointCloud2FieldsByString(1, "xyz");
	}

	sensor_msgs::PointCloud2Iterator<float> iter_x(res, "x");
	for (auto const& p : ufo::get<0>(a)) {
		iter_x[0] = static_cast<float>(p.x);
		iter_x[1] = static_cast<float>(p.y);
		if constexpr (3 <= Dim) {
			iter_x[2] = static_cast<float>(p.z);
		} else {
			iter_x[2] = 0.0f;
		}
		++iter_x;
	}

	if constexpr (ufo::contains_type_v<ufo::Color, Rest...>) {
		sensor_msgs::PointCloud2Iterator<std::uint8_t> iter_r(res, "r");

		for (auto const& c : ufo::get<ufo::Color>(a)) {
			iter_r[0] = static_cast<std::uint8_t>(c.red);
			iter_r[1] = static_cast<std::uint8_t>(c.green);
			iter_r[2] = static_cast<std::uint8_t>(c.blue);
			iter_r[3] = static_cast<std::uint8_t>(c.alpha);
			++iter_r;
		}
	}

	return res;
}

}  // namespace ufo_ros

#endif  // UFO_ROS_UFO_ROS_HPP