#ifndef UFO_ROS_UFO_ROS_HPP
#define UFO_ROS_UFO_ROS_HPP

// UFO
#include <ufo/cloud/point_cloud.hpp>
#include <ufo/math/quat.hpp>
#include <ufo/math/transform.hpp>
#include <ufo/vision/color.hpp>

// UFO ROS
#include <ufo_interfaces/msg/map.hpp>
#include <ufo_interfaces/msg/nav_map.hpp>

// STL
#include <cstddef>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
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

template <class Map>
void fromMsg(ufo_interfaces::msg::Map const& msg, Map& out, bool propagate = true)
{
	if (msg.data.empty()) {
		return;
	}

	// TODO: Implement
}

template <std::size_t Dim>
void fromMsg(ufo_interfaces::msg::NavMap const& msg)
{
	// TODO: Implement
}

template <class T = float>
[[nodiscard]] ufo::Vec3<T> fromMsg(geometry_msgs::msg::Point const& msg)
{
	return ufo::Vec3<T>(static_cast<T>(msg.x), static_cast<T>(msg.y),
	                    static_cast<T>(msg.z));
}

template <class T = float>
[[nodiscard]] ufo::Vec3<T> fromMsg(geometry_msgs::msg::Vector3 const& msg)
{
	return ufo::Vec3<T>(static_cast<T>(msg.x), static_cast<T>(msg.y),
	                    static_cast<T>(msg.z));
}

template <class T = float>
[[nodiscard]] ufo::Quat<T> fromMsg(geometry_msgs::msg::Quaternion const& msg)
{
	return ufo::Quat<T>(static_cast<T>(msg.w), static_cast<T>(msg.x), static_cast<T>(msg.y),
	                    static_cast<T>(msg.z));
}

template <class T = float>
[[nodiscard]] ufo::Transform3<T> fromMsg(geometry_msgs::msg::Transform const& msg)
{
	return ufo::Transform3<T>(fromMsg<T>(msg.rotation), fromMsg<T>(msg.translation));
}

template <class T = float>
[[nodiscard]] ufo::Transform3<T> fromMsg(geometry_msgs::msg::Pose const& msg)
{
	return ufo::Transform3<T>(fromMsg<T>(msg.orientation), fromMsg<T>(msg.position));
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
	}

	// TODO: Implement
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

/**************************************************************************************
|                                                                                     |
|                                       To msg                                        |
|                                                                                     |
**************************************************************************************/

template <class Map>
ufo_interfaces::msg::Map toMsg(Map const& a)
{
	ufo_interfaces::msg::Map res;

	// TODO: Implement

	return res;
}

template <std::size_t Dim>
ufo_interfaces::msg::NavMap toMsg()
{
	// TODO: Implement
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