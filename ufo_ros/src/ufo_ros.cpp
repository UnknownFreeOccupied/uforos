#include <ufo_ros/ufo_ros.hpp>

namespace ufo_ros
{
std::optional<sensor_msgs::PointField> getField(sensor_msgs::PointCloud2 const& cloud,
                                                std::string const& field_name)
{
	auto idx = sensor_msgs::getPointCloud2FieldIndex(cloud, field_name);
	return 0 > idx ? std::nullopt
	               : std::optional<sensor_msgs::PointField>{cloud.fields[idx]};
}

// Msg

// TODO: Implement and rename(?)
// ufo::FileHeader msgToHeader(ufo_msgs::Map const& msg)
// {
// 	if (msg.data.empty()) {
// 		return ufo::FileHeader();
// 	}

// 	constexpr std::size_t max_header_size = 200;

// 	ufo::Buffer buffer;
// 	buffer.write(
// 	    msg.data.data(),
// 	    std::min(max_header_size,
// 	             msg.data.size() * sizeof(decltype(ufo_msgs::Map::data)::value_type)));

// 	return ufo::readHeader(buffer);
// }
}  // namespace ufo_ros