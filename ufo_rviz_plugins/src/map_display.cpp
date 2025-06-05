// UFO
#include <ufo/container/tree/render.hpp>
#include <ufo/vision/color_map.hpp>
#include <ufo_ros/ufo_ros.hpp>
#include <ufo_rviz_plugins/map_display.hpp>

// ROS
#include <rviz_common/logging.hpp>

// Ament
#include <ament_index_cpp/get_package_share_directory.hpp>

// OGRE
#include <OgreHighLevelGpuProgramManager.h>
#include <OgreMaterialManager.h>
#include <OgreResourceGroupManager.h>
#include <OgreTechnique.h>
#include <OgreViewport.h>

// STL
#include <sstream>

namespace ufo_rviz_plugins
{
MapDisplay::MapDisplay() : image_(0, 0), rays_(0, 0)
{
	RVIZ_COMMON_LOG_INFO_STREAM("Creating MapDisplay");

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

	setupResources();
}

MapDisplay::~MapDisplay()
{
	RVIZ_COMMON_LOG_INFO_STREAM("Destroying MapDisplay");

	// TODO: Implement
}

void MapDisplay::setupResources()
{
	RVIZ_COMMON_LOG_INFO_STREAM("Setting up MapDisplay resources");

	auto& rgm = Ogre::ResourceGroupManager::getSingleton();
	if (rgm.resourceGroupExists("ufo_rendering")) {
		return;
	}

	std::string const pkg =
	    ament_index_cpp::get_package_share_directory("ufo_rviz_plugins");

	rgm.createResourceGroup("ufo_rendering");
	rgm.addResourceLocation(pkg + "/ogre_media", "FileSystem", "ufo_rendering");
	rgm.addResourceLocation(pkg + "/ogre_media/materials", "FileSystem", "ufo_rendering");
	rgm.addResourceLocation(pkg + "/ogre_media/materials/scripts", "FileSystem",
	                        "ufo_rendering");
	rgm.addResourceLocation(pkg + "/ogre_media/materials/glsl", "FileSystem",
	                        "ufo_rendering");
	rgm.initialiseResourceGroup("ufo_rendering");
}

void MapDisplay::onInitialize()
{
	MFDClass::onInitialize();

	RVIZ_COMMON_LOG_INFO_STREAM("Initializing MapDisplay");

	// TODO: Implement

	std::stringstream  ss;
	static std::size_t count = 0;
	ss << "UFOMaterial " << count++;
	point_material_ = Ogre::MaterialManager::getSingleton().getByName(
	    "ufo_rviz_plugins/Point", "ufo_rendering");
	point_material_ = Ogre::MaterialPtr(point_material_)->clone(ss.str() + " Point");
	point_material_->load();

	static std::size_t instance = 0;
	mesh_ =
	    scene_manager_->createManualObject("UFOMap Mesh #" + std::to_string(instance++));
	scene_node_->attachObject(mesh_);
}

void MapDisplay::reset()
{
	MFDClass::reset();

	RVIZ_COMMON_LOG_INFO_STREAM("Resetting MapDisplay");

	// TODO: Implement

	map_.clear();
	mesh_->clear();
	// // FIXME: How to clear material?
	point_material_->unload();
	// point_material_.reset();
}

void MapDisplay::processMessage(ufo_interfaces::msg::Map::ConstSharedPtr const msg)
{
	RVIZ_COMMON_LOG_INFO_STREAM("Received map (update)");
	auto const t0 = std::chrono::high_resolution_clock::now();
	ufo_ros::fromMsg(*msg, map_);
	auto const t1 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> const ms = t1 - t0;
	RVIZ_COMMON_LOG_INFO_STREAM("fromMsg took:        " << ms.count() << " ms");
}

void MapDisplay::update(float wall_dt, float ros_dt)
{
	auto const t0 = std::chrono::high_resolution_clock::now();

	// TODO: Update status

	// auto pred = ufo::pred::Leaf() && (ufo::pred::Occupied() || ufo::pred::Free());
	auto pred = ufo::pred::Leaf() && (ufo::pred::Occupied());

	bool camera_updated = updateCamera();
	if (camera_updated) {
		rays_ = camera_.rays();
		if (image_.rows() != rays_.rows() || image_.cols() != rays_.cols()) {
			image_ = ufo::Image<ufo::TraceResult<3>>(rays_.rows(), rays_.cols());
		}
	}
	bool map_updated = map_.modified();
	if (camera_updated && map_updated) {
		map_.modifiedReset();

		map_.trace(ufo::execution::par, rays_.begin(), rays_.end(), image_.begin(), pred,
		           camera_.near_clip, camera_.far_clip);

		// image_ = ufo::render(ufo::execution::par, map_, camera_,
		//                      ufo::pred::Leaf() && ufo::pred::Occupied());

		color();
	} else if (map_updated) {
		auto mod = map_.trace(ufo::execution::par, rays_.begin(), rays_.end(),
		                      ufo::pred::Leaf() && ufo::pred::Modified(), camera_.near_clip,
		                      camera_.far_clip);

		std::vector<ufo::Ray3> mod_rays;
		std::vector<unsigned>  indices;
		mod_rays.reserve(rays_.size());
		indices.reserve(rays_.size());
		for (unsigned i{}; mod.size() > i; ++i) {
			if (map_.valid(mod[i].node)) {
				mod_rays.push_back(rays_[i]);
				indices.push_back(i);
			}
		}

		map_.modifiedReset();

		auto tmp = map_.trace(ufo::execution::par, mod_rays.begin(), mod_rays.end(), pred,
		                      camera_.near_clip, camera_.far_clip);

		ufo::for_each(ufo::execution::par, std::size_t(0), indices.size(),
		              [this, &indices, &tmp](std::size_t i) { image_[indices[i]] = tmp[i]; });

		if (!indices.empty()) {
			color();
		}

	} else if (camera_updated) {
		map_.trace(ufo::execution::par, rays_.begin(), rays_.end(), image_.begin(), pred,
		           camera_.near_clip, camera_.far_clip);
		// image_ = ufo::render(ufo::execution::par, map_, camera_,
		//                      ufo::pred::Leaf() && ufo::pred::Occupied());

		color();
	}

	auto const t1 = std::chrono::high_resolution_clock::now();

	// TODO: Implement

	std::chrono::duration<double, std::milli> const ms = t1 - t0;
	RVIZ_COMMON_LOG_INFO_STREAM("update took:         " << ms.count() << " ms");
}

void MapDisplay::color()
{
	auto const t0 = std::chrono::high_resolution_clock::now();

	if (mesh_->getCurrentVertexCount() != image_.size()) {
		mesh_->clear();

		point_material_->getTechnique(0)->getPass(0)->setPointAttenuation(false);
		point_material_->getTechnique(0)->getPass(0)->setPointSize(
		    1 * 3);  // TODO: Make this a property
		point_material_->getTechnique(0)->getPass(0)->setLightingEnabled(false);
		point_material_->getTechnique(0)->getPass(0)->setSceneBlending(
		    Ogre::SBT_TRANSPARENT_ALPHA);
		point_material_->getTechnique(0)->getPass(0)->setDepthCheckEnabled(true);
		point_material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(true);

		mesh_->setDynamic(true);

		mesh_->estimateVertexCount(image_.size());
		mesh_->begin(point_material_->getName(), Ogre::RenderOperation::OT_POINT_LIST,
		             "ufo_rendering");
	} else {
		mesh_->beginUpdate(0);
	}

	// auto const pts = points();
	// for (auto const& p : pts) {
	// 	mesh_->position(p.first);
	// 	mesh_->colour(p.second);
	// }

	float min_distance = std::numeric_limits<float>::max();
	float max_distance = std::numeric_limits<float>::lowest();

	for (auto const& [node, hit, distance] : image_) {
		if (ufo::TreeIndex::NULL_POS == node.pos) {
			continue;
		}

		min_distance = std::min(min_distance, distance);
		max_distance = std::max(max_distance, distance);

		// auto color = colorMap(ufo::color::BLACK_BODY, distance, prev_min_distance_,
		//                       prev_max_distance_);
		// color.alpha = 255;

		mesh_->position(toOgre(hit));
		if (map_.occupancyFree(node)) {
			mesh_->colour(0, 1, 0, 1);
		} else {
			mesh_->colour(toOgre(map_.color(node)));
		}
	}

	prev_min_distance_ = min_distance;
	prev_max_distance_ = max_distance;

	mesh_->end();

	auto const t1 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> const ms = t1 - t0;
	RVIZ_COMMON_LOG_INFO_STREAM("color took:          " << ms.count() << " ms");
}

std::vector<std::pair<Ogre::Vector3, Ogre::ColourValue>> MapDisplay::points() const
{
	float min_distance = std::numeric_limits<float>::max();
	float max_distance = std::numeric_limits<float>::lowest();
	for (auto const& [node, _, distance] : image_) {
		if (!map_.valid(node)) {
			continue;
		}

		min_distance = std::min(min_distance, distance);
		max_distance = std::max(max_distance, distance);
	}

	// std::vector<std::pair<Ogre::Vector3, Ogre::ColourValue>> res(image_.size());

	// ufo::transform(ufo::execution::par, image_.begin(), image_.end(), res.begin(),
	//                [this, min_distance, max_distance](auto const& v) {
	// 	               if (!map_.valid(v.node)) {
	// 		               return std::pair(Ogre::Vector3(0, 0, 0),
	// 		                                Ogre::ColourValue(0.0f, 0.0f, 0.0f, 0.0f));
	// 	               }

	// 	               // auto color = map_.color(v.node);

	// 	               // if (map_.voidRegion(v.node)) {
	// 	               //   return std::pair(toOgre(v.hit), Ogre::ColourValue(0.0f,
	// 	               //   0.0f, 1.0f, 1.0f));
	// 	               // } else if (map_.occupancyFree(v.node)) {
	// 	               //   return std::pair(toOgre(v.hit), Ogre::ColourValue(0.0f, 1.0f,
	// 	               //   0.0f, 1.0f));
	// 	               // } else {
	// 	               //   return std::pair(toOgre(v.hit),
	// 	               //                    toOgre(colorMap(ufo::color::BLACK_BODY,
	// 	               //                    v.distance,
	// 	               //                                    min_distance, max_distance)));
	// 	               // }

	// 	               auto color = colorMap(ufo::color::BLACK_BODY, v.distance,
	// min_distance, 	                                     max_distance);

	// 	               return std::pair(toOgre(v.hit), toOgre(color));
	//                });

	std::vector<std::pair<Ogre::Vector3, Ogre::ColourValue>> res;
	res.reserve(image_.size());

	for (auto const& [node, hit, distance] : image_) {
		if (!map_.valid(node)) {
			continue;
		}

		auto color = map_.color(node);

		// if (map_.voidRegion(node)) {
		// 	color = ufo::Color(0, 0, 255, 255);
		// } else if (map_.occupancyFree(node)) {
		// 	color = ufo::Color(0, 255, 0, 255);
		// } else {
		color = colorMap(ufo::color::BLACK_BODY, distance, min_distance, max_distance);
		// }

		res.emplace_back(toOgre(hit), toOgre(color));
	}

	return res;
}

bool MapDisplay::updateCamera()
{
	Ogre::Viewport* viewport = scene_manager_->getCurrentViewport();
	if (!viewport) {
		return false;
	}

	// FIXME: This happens when the mouse moves within the viewport without any mouse button
	// being pressed.
	if (1 >= viewport->getActualHeight() || 1 >= viewport->getActualWidth()) {
		return false;
	}

	// auto cameras = scene_manager_->getCameras();
	// for (auto [n, _] : cameras) {
	// 	RVIZ_COMMON_LOG_INFO_STREAM("Camera: " << n);
	// }

	Ogre::Camera* camera = viewport->getCamera();
	if (!camera) {
		return false;
	}

	auto const prev = camera_;

	Ogre::Vector3 const&    position    = camera->getPositionForViewUpdate();
	Ogre::Quaternion const& orientation = camera->getOrientationForViewUpdate();
	// Ogre::Vector3 const     up          = camera->getRealUp();
	Ogre::Vector3 const up = orientation.zAxis();

	camera_.pose = ufo::Transform3f(
	    ufo::Quatf(orientation.w, orientation.x, orientation.y, orientation.z),
	    ufo::Vec3f(position.x, position.y, position.z));
	camera_.pose = ufo::inverse(camera_.pose);

	std::size_t full_rows = viewport->getActualHeight();
	std::size_t full_cols = viewport->getActualWidth();
	camera_.rows          = full_rows * 0.34;  // * options.resolution_factor;
	camera_.cols          = full_cols * 0.34;  // * options.resolution_factor;

	camera_.vertical_fov    = camera->getFOVy().valueRadians();
	camera_.near_clip       = camera->getNearClipDistance();
	camera_.far_clip        = camera->getFarClipDistance();
	camera_.zoom            = 0.0f;  // TODO: Implement zoom
	camera_.up              = ufo::Vec3f(up.x, up.y, up.z);
	camera_.projection_type = ufo::ProjectionType::PERSPECTIVE;

	return prev != camera_;
}

[[nodiscard]] Ogre::Vector3 MapDisplay::toOgre(ufo::Vec3f const& v) const
{
	return Ogre::Vector3(v.x, v.y, v.z);
}

[[nodiscard]] Ogre::ColourValue MapDisplay::toOgre(ufo::Color const& color) const
{
	return Ogre::ColourValue(color.redf(), color.greenf(), color.bluef(), color.alphaf());
}

void MapDisplay::updateEdgeWidth()
{
	float edge_width = edge_width_property_->getFloat();

	// TODO: Implement

	RVIZ_COMMON_LOG_INFO_STREAM("Updating edge width to " << edge_width << " meters");

	context_->queueRender();
}

void MapDisplay::updateEdgeStyle()
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
PLUGINLIB_EXPORT_CLASS(ufo_rviz_plugins::MapDisplay, rviz_common::Display)