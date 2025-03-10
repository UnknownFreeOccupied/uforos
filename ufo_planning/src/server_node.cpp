// STL
#include <memory>

// ROS
#include <rclcpp/rclcpp.hpp>

// UFO ROS
#include <ufo_planning/server.hpp>

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<ufo_planning::PlanningServer<DIMENSIONS>>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}