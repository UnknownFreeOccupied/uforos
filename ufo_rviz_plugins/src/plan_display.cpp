// UFO
#include <ufo_rviz_plugins/plan_display.hpp>

// ROS
#include <rviz_common/logging.hpp>

namespace ufo_rviz_plugins {
void PlanDisplay::processMessage(
    ufo_interfaces::msg::Plan::ConstSharedPtr const msg) {
  RVIZ_COMMON_LOG_INFO_STREAM("Received plan with " << msg->header.frame_id
                                                    << " waypoints");
}
} // namespace ufo_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ufo_rviz_plugins::PlanDisplay, rviz_common::Display)