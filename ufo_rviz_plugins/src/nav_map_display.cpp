// UFO
#include <ufo_rviz_plugins/nav_map_display.hpp>

// ROS
#include <rviz_common/logging.hpp>

namespace ufo_rviz_plugins
{
NavMapDisplay::NavMapDisplay()
{
	RVIZ_COMMON_LOG_INFO_STREAM("Creating NavMapDisplay");

	edge_width_property_ = new rviz_common::properties::FloatProperty(
	    "Edge Width", 0.03f, "The width, in meters, of each edge.", this,
	    SLOT(updateEdgeWidth()), this);
	edge_width_property_->setMin(0.001);

	edge_style_property_ = new rviz_common::properties::EnumProperty(
	    "Edge Style", "Lines", "The rendering operation to use to draw the edges.", this,
	    SLOT(updateEdgeStyle()));
	edge_style_property_->addOption("Lines", LINES);
	edge_style_property_->addOption("Arrows", ARROWS);

	// TODO: Implement
}

NavMapDisplay::~NavMapDisplay()
{
	RVIZ_COMMON_LOG_INFO_STREAM("Destroying NavMapDisplay");

	// TODO: Implement
}

void NavMapDisplay::onInitialize()
{
	MFDClass::onInitialize();

	RVIZ_COMMON_LOG_INFO_STREAM("Initializing NavMapDisplay");

	// TODO: Implement
}

void NavMapDisplay::reset()
{
	MFDClass::reset();

	RVIZ_COMMON_LOG_INFO_STREAM("Resetting NavMapDisplay");

	// TODO: Implement
}

void NavMapDisplay::processMessage(ufo_interfaces::msg::NavMap::ConstSharedPtr const msg)
{
	RVIZ_COMMON_LOG_INFO_STREAM("Received plan with " << msg->header.frame_id
	                                                  << " waypoints");

	// TODO: Implement
}

void NavMapDisplay::updateEdgeWidth()
{
	float edge_width = edge_width_property_->getFloat();

	// TODO: Implement

	RVIZ_COMMON_LOG_INFO_STREAM("Updating edge width to " << edge_width << " meters");

	context_->queueRender();
}

void NavMapDisplay::updateEdgeStyle()
{
	auto style = static_cast<EdgeStyle>(edge_style_property_->getOptionInt());

	if (LINES == style) {
		RVIZ_COMMON_LOG_INFO_STREAM("Updating edge style to lines");
	} else if (ARROWS == style) {
		RVIZ_COMMON_LOG_INFO_STREAM("Updating edge style to arrows");
	}

	// TODO: Implement
}
}  // namespace ufo_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ufo_rviz_plugins::NavMapDisplay, rviz_common::Display)