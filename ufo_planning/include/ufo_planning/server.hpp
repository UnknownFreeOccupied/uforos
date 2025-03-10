#ifndef UFO_PLANNING_SERVER_HPP
#define UFO_PLANNING_SERVER_HPP

// UFO
#include <ufo/map/ufomap.hpp>
#include <ufo/plan/nav_map.hpp>

// UFO ROS
#include <ufo_interfaces/msg/map.hpp>
#include <ufo_interfaces/msg/nav_map.hpp>

// STL
#include <cstddef>
#include <functional>
#include <memory>

// ROS
#include <nav_msgs/srv/get_plan.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ufo_planning
{
template <std::size_t Dim>
class PlanningServer : public rclcpp::Node
{
 public:
	/**
	 * @brief A constructor for ufo_planning::PlanningServer
	 * @param options Additional options to control creation of the node.
	 */
	PlanningServer(rclcpp::NodeOptions const& options = rclcpp::NodeOptions())
	    : rclcpp::Node("planning_server_" + std::to_string(Dim) + "d", options)
	{
		plan_service_ = create_service<nav_msgs::srv::GetPlan>(
		    "get_plan", std::bind(&PlanningServer::getPlan, this, std::placeholders::_1,
		                          std::placeholders::_2));

		nav_map_publisher_ = create_publisher<ufo_interfaces::msg::NavMap>("nav_map", 10);

		map_subscription_ = create_subscription<ufo_interfaces::msg::Map>(
		    "map", 10, [this](ufo_interfaces::msg::Map::SharedPtr msg) { mapCallback(msg); });
	}

 private:
	void getPlan(std::shared_ptr<nav_msgs::srv::GetPlan::Request> const /* request */,
	             std::shared_ptr<nav_msgs::srv::GetPlan::Response> /* response */)
	{
		// TODO: Implement
	}

	void mapCallback(ufo_interfaces::msg::Map::SharedPtr const /* msg */)
	{
		// TODO: Implement
	}

 private:
	ufo::NavMap<Dim>                 nav_map_;
	ufo::Map<Dim, ufo::OccupancyMap> map_;

	rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr        plan_service_;
	rclcpp::Publisher<ufo_interfaces::msg::NavMap>::SharedPtr nav_map_publisher_;
	rclcpp::Subscription<ufo_interfaces::msg::Map>::SharedPtr map_subscription_;
};
}  // namespace ufo_planning

#endif  // UFO_PLANNING_SERVER_HPP