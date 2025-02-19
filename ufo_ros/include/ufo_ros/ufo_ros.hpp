#ifndef UFOROS_CONVERSIONS_H
#define UFOROS_CONVERSIONS_H

// ROS
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>

// UFO
#include <ufo/cloud/cloud.hpp>
#include <ufo/math/transform3.hpp>
#include <ufo/vision/color.hpp>

// UFOROS
#include <ufo_msgs/Map.h>

// STL
#include <optional>
#include <string>

namespace ufo_ros
{
std::optional<sensor_msgs::PointField> getField(sensor_msgs::PointCloud2 const& cloud,
                                                std::string const& field_name);

// Point clouds
template <class... P>
void fromMsg(sensor_msgs::PointCloud2 const& cloud_in, ufo::Cloud<P...>& cloud_out,
              bool filter_nan = true)
{
	if (0 == cloud_in.point_step || 0 == cloud_in.row_step || 0 == cloud_in.height) {
		throw std::runtime_error("cloud_in point_step, height, and/or row_step is zero");
	}

	// for (int i = 0; i < cloud_in.fields.size(); ++i) {
	// 	std::cout <<  cloud_in.fields[i].name.c_str() << std::endl;;
	// }

	// Get all fields
	auto x_field         = getField(cloud_in, "x");
	auto y_field         = getField(cloud_in, "y");
	auto z_field         = getField(cloud_in, "z");
	auto rgb_field       = getField(cloud_in, "rgb");
	rgb_field            = rgb_field ? rgb_field : getField(cloud_in, "rgba");
	auto label_field     = getField(cloud_in, "label");
	label_field          = label_field ? label_field : getField(cloud_in, "l");
	auto value_field     = getField(cloud_in, "value");
	value_field          = value_field ? value_field : getField(cloud_in, "v");
	auto intensity_field = getField(cloud_in, "intensity");

	// There must be x,y,z values
	if (!x_field || !y_field || !z_field) {  // FIXME: Need to check if the are consecutive?
		throw std::runtime_error("cloud_in missing one or more of the xyz fields");
	}

	// FIXME: Add different depending on the type
	// Perhaps make them variants?
	sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in, "x");
	std::optional<sensor_msgs::PointCloud2ConstIterator<std::uint8_t>> iter_rgb;
	// std::optional<sensor_msgs::PointCloud2ConstIterator<std::uint32_t>> iter_label;
	// std::optional<sensor_msgs::PointCloud2ConstIterator<float>>         iter_value;
	// std::optional<sensor_msgs::PointCloud2ConstIterator<float>>         iter_intensity;

	// Create optional iteraters if wanted and available
	// if constexpr ((ufo::is_color_v<P> || ...)) {
	if constexpr ((ufo::contains_type_v<ufo::Color, P> || ...)) {
		if (rgb_field) {
			iter_rgb.emplace(cloud_in, rgb_field->name);
		}
	}

	// if constexpr ((ufo::is_semantic_v<P> || ...)) {
	// 	if (label_field) {
	// 		iter_label.emplace(cloud_in, label_field->name);
	// 	}
	// 	if (value_field) {
	// 		iter_value.emplace(cloud_in, value_field->name);
	// 	}
	// }
	// if constexpr ((ufo::is_intensity_v<P> || ...)) {
	// 	if (intensity_field) {
	// 		iter_intensity.emplace(cloud_in, value_field->name);
	// 	}
	// }

	// Preallocate
	std::size_t index = cloud_out.size();

	cloud_out.resize(index + (cloud_in.data.size() / cloud_in.point_step));

	// Copy data
	for (; iter_x.end() != iter_x; ++iter_x) {
		if (!filter_nan ||
		    (!std::isnan(iter_x[0]) && !std::isnan(iter_x[1]) && !std::isnan(iter_x[2]))) {
			auto& points    = cloud_out.template get<ufo::Vec3f>();
			points[index].x = iter_x[0];
			points[index].y = iter_x[1];
			points[index].z = iter_x[2];

			if constexpr ((ufo::contains_type_v<ufo::Color, P> || ...)) {
				if (iter_rgb) {
					auto& colors        = cloud_out.template get<ufo::Color>();
					colors[index].red   = (*iter_rgb)[0];
					colors[index].green = (*iter_rgb)[1];
					colors[index].blue  = (*iter_rgb)[2];
					// TODO: What about alpha?
				}
			}
			// if constexpr ((ufo::is_semantic_v<P> || ...)) {
			// 	if (iter_label) {
			// 		cloud_out[index].label = *(*iter_label);
			// 	}
			// 	if (iter_value) {
			// 		cloud_out[index].value = *(*iter_value);
			// 	}
			// }
			// if constexpr ((ufo::is_intensity_v<P> || ...)) {
			// 	if (intensity_field) {
			// 		cloud_out[index].intesity = *(*iter_intensity);
			// 	}
			// }
			++index;
		}

		// Increment optional iterators
		if constexpr ((ufo::contains_type_v<ufo::Color, P> || ...)) {
			if (iter_rgb) {
				++(*iter_rgb);
			}
		}
		// if constexpr ((ufo::is_semantic_v<P> || ...)) {
		// 	if (iter_label) {
		// 		++(*iter_label);
		// 	}
		// 	if (iter_value) {
		// 		++(*iter_value);
		// 	}
		// }
		// if constexpr ((ufo::is_intensity_v<P> || ...)) {
		// 	if (iter_intensity) {
		// 		++(*iter_intensity);
		// 	}
		// }
	}

	// Resize
	cloud_out.resize(index);
}

constexpr ufo::Transform3f fromMsg(geometry_msgs::Transform const& transform)
{
	return {ufo::Quatf{static_cast<float>(transform.rotation.w),
	                   static_cast<float>(transform.rotation.x),
	                   static_cast<float>(transform.rotation.y),
	                   static_cast<float>(transform.rotation.z)},
	        ufo::Vec3f{static_cast<float>(transform.translation.x),
	                   static_cast<float>(transform.translation.y),
	                   static_cast<float>(transform.translation.z)}};
}

constexpr ufo::Transform3f fromMsg(geometry_msgs::Pose const& pose)
{
	return {
	    ufo::Quatf{
	        static_cast<float>(pose.orientation.w), static_cast<float>(pose.orientation.x),
	        static_cast<float>(pose.orientation.y), static_cast<float>(pose.orientation.z)},
	    ufo::Vec3f{static_cast<float>(pose.position.x), static_cast<float>(pose.position.y),
	               static_cast<float>(pose.position.z)}};
}

// Vector3/Point
template <typename T>
constexpr void fromMsg(geometry_msgs::Point const& point_in, ufo::Vec3<T>& point_out)
{
	point_out.x = point_in.x;
	point_out.y = point_in.y;
	point_out.z = point_in.z;
}

template <typename T>
constexpr void fromMsg(geometry_msgs::Vector3 const& point_in, ufo::Vec3<T>& point_out)
{
	point_out.x = point_in.x;
	point_out.y = point_in.y;
	point_out.z = point_in.z;
}

constexpr ufo::Vec3d fromMsg(geometry_msgs::Point const& point)
{
	return {point.x, point.y, point.z};
}

constexpr ufo::Vec3d fromMsg(geometry_msgs::Vector3 const& point)
{
	return {point.x, point.y, point.z};
}

template <typename T>
constexpr void toMsg(ufo::Vec3<T> const& point_in, geometry_msgs::Point& point_out)
{
	point_out.x = point_in.x;
	point_out.y = point_in.y;
	point_out.z = point_in.z;
}

template <typename T>
constexpr void toMsg(ufo::Vec3<T> const& point_in, geometry_msgs::Vector3& point_out)
{
	point_out.x = point_in.x;
	point_out.y = point_in.y;
	point_out.z = point_in.z;
}

template <typename P, typename T>
constexpr P toMsg(ufo::Vec3<T> const& point)
{
	P p;
	toMsg(point, p);
	return p;
}


// TODO: Implement below

// //
// // ROS message type to UFO type
// //

// // ufo::FileHeader msgToHeader(ufo_msgs::Map const& msg);

// template <class Map>
// void fromMsg(ufo_msgs::Map const& msg, Map& map, bool propagate = true)
// {
// 	if (msg.data.empty()) {
// 		return;
// 	}
// 	ufo::Buffer buffer;
// 	buffer.write(msg.data.data(),
// 	             msg.data.size() * sizeof(decltype(ufo_msgs::Map::data)::value_type));
// 	map.read(buffer, propagate);
// }

// //
// // UFO type to ROS message type
// //

// template <class Map, class Predicates,
//           typename = std::enable_if_t<!std::is_scalar_v<Predicates>>>
// decltype(ufo_msgs::Map::data) toMsg(Map const& map, Predicates const& predicates,
//                                       unsigned int depth = 0, bool compress = false,
//                                       ufo::mt_t map_types                      = 0,
//                                       int       compression_acceleration_level = 1,
//                                       int       compression_level              = 0)
// {
// 	auto                         data = map.write(predicates, depth, compress, map_types,
// 	                                              compression_acceleration_level, compression_level);
// 	decltype(ufo_msgs::Map::data) ret;
// 	ret.resize(data.size());
// 	data.read(ret.data(), data.size());
// 	return ret;
// }

// template <class Map>
// decltype(ufo_msgs::Map::data) toMsg(Map const& map, unsigned int depth = 0,
//                                       bool compress = false, ufo::mt_t map_types = 0,
//                                       int compression_acceleration_level = 1,
//                                       int compression_level              = 0)
// {
// 	auto data = map.write(depth, compress, map_types, compression_acceleration_level,
// 	                      compression_level);
// 	decltype(ufo_msgs::Map::data) ret;
// 	ret.resize(data.size());
// 	data.read(ret.data(), data.size());
// 	return ret;
// }

// template <class Map>
// decltype(ufo_msgs::Map::data) toMsgModified(Map& map, bool compress = false,
//                                               ufo::mt_t map_types                = 0,
//                                               int compression_acceleration_level = 1,
//                                               int compression_level              = 0)
// {
// 	auto data = map.writeModified(compress, map_types, compression_acceleration_level,
// 	                              compression_level);
// 	decltype(ufo_msgs::Map::data) ret;
// 	ret.resize(data.size());
// 	data.read(ret.data(), data.size());
// 	return ret;
// }

// template <class Map>
// decltype(ufo_msgs::Map::data) toMsgResetModified(Map& map, bool compress = false,
//                                                    ufo::mt_t map_types                = 0,
//                                                    int compression_acceleration_level = 1,
//                                                    int compression_level              = 0)
// {
// 	auto data = map.writeModifiedAndReset(
// 	    compress, map_types, compression_acceleration_level, compression_level);
// 	decltype(ufo_msgs::Map::data) ret;
// 	ret.resize(data.size());
// 	data.read(ret.data(), data.size());
// 	return ret;
// }

}  // namespace ufo_ros

#endif