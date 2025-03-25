#ifndef UFO_ROS_UFO_ROS_HPP
#define UFO_ROS_UFO_ROS_HPP

// UFO
#include <ufo/cloud/point_cloud.hpp>
#include <ufo/core/label.hpp>
#include <ufo/math/quat.hpp>
#include <ufo/math/transform.hpp>
#include <ufo/vision/color.hpp>

// UFO ROS
// #include <ufo_interfaces/msg/map.hpp>
// #include <ufo_interfaces/msg/nav_map.hpp>
#include <ufo_msgs/Map.h>

// STL
#include <cstddef>

// ROS
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_field_conversion.h>

namespace ufo_ros
{
/**************************************************************************************
|                                                                                     |
|                                      From msg                                       |
|                                                                                     |
**************************************************************************************/

template <class Map>
void fromMsg(ufo_msgs::Map const& msg, Map& out, bool propagate = true)
{
	if (msg.data.empty()) {
		return;
	}

	// TODO: Implement
}

// template <std::size_t Dim>
// void fromMsg(ufo_msgs::NavMap const& msg)
// {
// 	// TODO: Implement
// }

/*!
 * @brief Convert a Point message to its equivalent ufo representation.
 *
 * @param msg A Point message.
 * @return The converted Point message as a ufo Vec3.
 */
template <class T = float>
[[nodiscard]] ufo::Vec3<T> fromMsg(geometry_msgs::Point const& msg)
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
[[nodiscard]] ufo::Vec3<T> fromMsg(geometry_msgs::Vector3 const& msg)
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
[[nodiscard]] ufo::Quat<T> fromMsg(geometry_msgs::Quaternion const& msg)
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
[[nodiscard]] ufo::Transform3<T> fromMsg(geometry_msgs::Transform const& msg)
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
[[nodiscard]] ufo::Transform3<T> fromMsg(geometry_msgs::Pose const& msg)
{
	return ufo::Transform3<T>(fromMsg<T>(msg.orientation), fromMsg<T>(msg.position));
}

template <std::size_t Dim, class T, class... Rest>
void fromMsg(sensor_msgs::PointCloud2 const& msg, ufo::PointCloud<Dim, T, Rest...>& out)
{
	// for (int i = 0; i < msg.fields.size(); ++i) {
	// 	std::cout << msg.fields[i].name.c_str() << std::endl;
	// }

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

	if constexpr (ufo::contains_type_v<ufo::Label, Rest...>) {
		// TODO: Check if 'label' or 'l'
		sensor_msgs::PointCloud2ConstIterator<ufo::label_t> iter_l(msg, "label");

		for (auto& l : ufo::get<ufo::Label>(out)) {
			l = *iter_l;

			++iter_l;
		}
	}

	// TODO: Implement
}

template <std::size_t Dim, class T = float, class... Ts>
[[nodiscard]] ufo::PointCloud<Dim, T, Ts...> fromMsg(sensor_msgs::PointCloud2 const& msg)
{
	ufo::PointCloud<Dim, T, Ts...> cloud;
	fromMsg(msg, cloud);
	return cloud;
}

template <class... Ts>
[[nodiscard]] ufo::PointCloud<3, float, Ts...> fromMsg(
    sensor_msgs::PointCloud2 const& msg)
{
	return fromMsg<3, float, Ts...>(msg);
}

/**************************************************************************************
|                                                                                     |
|                                       To msg                                        |
|                                                                                     |
**************************************************************************************/

template <class Map>
ufo_msgs::Map toMsg(Map const& a)
{
	ufo_msgs::Map res;

	// TODO: Implement

	return res;
}

// template <std::size_t Dim>
// ufo_msgs::NavMap toMsg()
// {
// 	// TODO: Implement
// }

// TODO: Do something with point?

// template <class T>
// [[nodiscard]] geometry_msgs::Vector3 toMsg(ufo::Vec3<T> const& a)
// {
// 	geometry_msgs::Vector3 res;
// 	res.x = a.x;
// 	res.y = a.y;
// 	res.z = a.z;
// 	return res;
// }

template <class T>
[[nodiscard]] geometry_msgs::Point toMsg(ufo::Vec3<T> const& a)
{
	geometry_msgs::Point res;
	res.x = a.x;
	res.y = a.y;
	res.z = a.z;
	return res;
}

template <class T>
[[nodiscard]] geometry_msgs::Quaternion toMsg(ufo::Quat<T> const& a)
{
	geometry_msgs::Quaternion res;
	res.w = a.w;
	res.x = a.x;
	res.y = a.y;
	res.z = a.z;
	return res;
}

template <class T>
[[nodiscard]] geometry_msgs::Transform toMsg(ufo::Transform3<T> const& a)
{
	geometry_msgs::Transform res;
	res.rotation    = toMsg(a.rotation);
	res.translation = toMsg(a.translation);
	return res;
}

// TODO: Do something with pose?

template <std::size_t Dim, class T, class... Rest>
[[nodiscard]] sensor_msgs::PointCloud2 toMsg(ufo::PointCloud<Dim, T, Rest...> const& a)
{
	sensor_msgs::PointCloud2 res;
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