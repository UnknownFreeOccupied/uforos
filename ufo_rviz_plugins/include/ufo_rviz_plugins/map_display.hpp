#ifndef UFO_RVIZ_PLUGINS_MAP_DISPLAY_HPP
#define UFO_RVIZ_PLUGINS_MAP_DISPLAY_HPP

// UFO
#include <ufo/geometry/frustum.hpp>
#include <ufo/map/ufomap.hpp>
#include <ufo/vision/camera.hpp>
#include <ufo/vision/image.hpp>
#include <ufo_interfaces/msg/map.hpp>

// ROS
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

// OGRE
#include <OgreColourValue.h>
#include <OgreManualObject.h>
#include <OgreMaterial.h>
#include <OgreVector3.h>

// STL
#include <utility>
#include <vector>

namespace ufo_rviz_plugins
{
class MapDisplay : public rviz_common::MessageFilterDisplay<ufo_interfaces::msg::Map>
{
	Q_OBJECT

 public:
	MapDisplay();

	~MapDisplay() override;

 protected:
	void setupResources();

	void onInitialize() override;

	void reset() override;

	void processMessage(ufo_interfaces::msg::Map::ConstSharedPtr const msg) override;

	void update(float wall_dt, float ros_dt) override;

	void color();

	[[nodiscard]] std::vector<std::pair<Ogre::Vector3, Ogre::ColourValue>> points() const;

	bool updateCamera();

	[[nodiscard]] Ogre::Vector3 toOgre(ufo::Vec3f const& v) const;

	[[nodiscard]] Ogre::ColourValue toOgre(ufo::Color const& color) const;

 private Q_SLOTS:
	void updateEdgeWidth();

	void updateEdgeStyle();

 private:
	// TODO: Variable dim
	ufo::Map<3, ufo::OccupancyMap, ufo::ColorMap, ufo::VoidRegionMap> map_;

	ufo::Camera camera_;
	// TODO: Variable dim
	ufo::Image<ufo::Ray<3>> rays_;

	// TODO: Variable dim
	ufo::Image<ufo::TraceResult<3>> image_;
	Ogre::ManualObject*             mesh_;
	Ogre::MaterialPtr               point_material_;

	float prev_min_distance_ = 0.0f;
	float prev_max_distance_ = 10.0f;

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

#endif  // UFO_RVIZ_PLUGINS_MAP_DISPLAY_HPP