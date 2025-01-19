#include "colorized_mesh_display/colorized_mesh_visual.h"
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <boost/lexical_cast.hpp>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

static uint32_t count = 0;

namespace colorized_mesh_display
{

ColorizedMeshVisual::ColorizedMeshVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node)
  : scene_manager_(scene_manager), material_name_("BaseWhiteNoLighting")
{
  if (!parent_node)
  {
    parent_node = scene_manager_->getRootSceneNode();
  }

  frame_node_ = parent_node->createChildSceneNode();
  manual_object_ = scene_manager_->createManualObject("ColorizedMesh" + boost::lexical_cast<std::string>(count++));
}

ColorizedMeshVisual::~ColorizedMeshVisual()
{
  scene_manager_->destroyManualObject(manual_object_);
  scene_manager_->destroySceneNode(frame_node_);
}

void ColorizedMeshVisual::visualizeMesh(const pcl::PolygonMesh &mesh)
{
  manual_object_->clear();

  auto color_it = std::find_if(
      mesh.cloud.fields.begin(), mesh.cloud.fields.end(),
      [](const pcl::PCLPointField &field) { return field.name == "rgb" || field.name == "rgba"; });

  pcl::PointCloud<pcl::PointXYZRGBNormal> vertex_cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, vertex_cloud);

  manual_object_->estimateVertexCount(vertex_cloud.size());
  manual_object_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);

  for (const pcl::PointXYZRGBNormal &vertex : vertex_cloud)
  {
    manual_object_->position(vertex.x, vertex.y, vertex.z);
    manual_object_->normal(vertex.normal_x, vertex.normal_y, vertex.normal_z);

    float r = 0.5f, g = 0.5f, b = 0.5f;
    if (color_it != mesh.cloud.fields.end())
    {
      r = static_cast<float>(vertex.r) / 255.0f;
      g = static_cast<float>(vertex.g) / 255.0f;
      b = static_cast<float>(vertex.b) / 255.0f;
    }

    manual_object_->colour(r, g, b);
  }

  for (const auto &polygon : mesh.polygons)
  {
    for (std::size_t j = 0; j < polygon.vertices.size() - 2; ++j)
    {
      manual_object_->triangle(polygon.vertices[0], polygon.vertices[j + 1], polygon.vertices[j + 2]);
      manual_object_->triangle(polygon.vertices[j + 2], polygon.vertices[j + 1], polygon.vertices[0]);
    }
  }

  manual_object_->end();
  frame_node_->attachObject(manual_object_);
}

void ColorizedMeshVisual::setFramePosition(const Ogre::Vector3 &position)
{
  frame_node_->setPosition(position);
}

void ColorizedMeshVisual::setFrameOrientation(const Ogre::Quaternion &orientation)
{
  frame_node_->setOrientation(orientation);
}

} // namespace colorized_mesh_display