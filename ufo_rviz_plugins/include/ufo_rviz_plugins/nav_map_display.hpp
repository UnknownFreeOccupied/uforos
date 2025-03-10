#ifndef UFO_RVIZ_PLUGINS_NAV_MAP_DISPLAY_HPP
#define UFO_RVIZ_PLUGINS_NAV_MAP_DISPLAY_HPP

// UFO
#include <ufo/plan/nav_map.hpp>
#include <ufo_interfaces/msg/nav_map.hpp>

// ROS
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

// STL
#include <vector>

namespace ufo_rviz_plugins
{
class NavMapDisplay
    : public rviz_common::MessageFilterDisplay<ufo_interfaces::msg::NavMap>
{
	Q_OBJECT

 public:
	NavMapDisplay();

	~NavMapDisplay() override;

 protected:
	void onInitialize() override;

	void reset() override;

	void processMessage(ufo_interfaces::msg::NavMap::ConstSharedPtr const msg) override;

 private Q_SLOTS:
	void updateEdgeWidth();

	void updateEdgeStyle();

 private:
	rviz_rendering::BillboardLine*          edge_lines_{};
	std::vector<rviz_rendering::Arrow*>     edge_arrows_{};
	rviz_common::properties::FloatProperty* edge_width_property_{};
	rviz_common::properties::EnumProperty*  edge_style_property_{};
	rviz_common::properties::FloatProperty* edge_color_style_property_{};
	rviz_common::properties::ColorProperty* edge_color_property_{};
	rviz_common::properties::FloatProperty* edge_alpha_property_{};

	enum EdgeStyle { LINES, ARROWS };
	enum EdgeColorStyle { FIXED, COST };

	rviz_common::properties::VectorProperty* offset_property_{};
};
}  // namespace ufo_rviz_plugins

#endif  // UFO_RVIZ_PLUGINS_NAV_MAP_DISPLAY_HPP