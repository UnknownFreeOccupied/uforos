#include <ufo_ros/conversions.hpp>

namespace ufo_ros
{
std::optional<sensor_msgs::PointField> getField(sensor_msgs::PointCloud2 const& cloud,
                                                std::string const& field_name)
{
	auto idx = sensor_msgs::getPointCloud2FieldIndex(cloud, field_name);
	return 0 > idx ? std::nullopt
	               : std::optional<sensor_msgs::PointField>{cloud.fields[idx]};
}
}  // namespace ufo_ros