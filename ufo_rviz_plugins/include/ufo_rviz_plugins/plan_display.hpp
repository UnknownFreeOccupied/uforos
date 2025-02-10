#ifndef UFO_RVIZ_PLUGINS_PLAN_DISPLAY_HPP
#define UFO_RVIZ_PLUGINS_PLAN_DISPLAY_HPP

// UFO
#include <ufo_interfaces/msg/plan.hpp>

// ROS
#include <rviz_common/message_filter_display.hpp>

namespace ufo_rviz_plugins {
class PlanDisplay
    : public rviz_common::MessageFilterDisplay<ufo_interfaces::msg::Plan> {
  Q_OBJECT

protected:
  void
  processMessage(ufo_interfaces::msg::Plan::ConstSharedPtr const msg) override;
};
} // namespace ufo_rviz_plugins

#endif // UFO_RVIZ_PLUGINS_PLAN_DISPLAY_HPP