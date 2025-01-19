#ifndef COLORIZED_MESH_VISUAL_HPP
#define COLORIZED_MESH_VISUAL_HPP

#include <pcl/PolygonMesh.h>
#include <OgreVector3.h>

namespace Ogre
{
// class Vector3;
class Quaternion;
class SceneNode;
class SceneManager;
class ManualObject;
class Entity;
}

namespace colorized_mesh_display
{

class ColorizedMeshVisual
{
public:
  ColorizedMeshVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node = nullptr);
  ~ColorizedMeshVisual();

  void visualizeMesh(const pcl::PolygonMesh &mesh);
  void setFramePosition(const Ogre::Vector3 &position);
  void setFrameOrientation(const Ogre::Quaternion &orientation);

private:
  Ogre::SceneManager *scene_manager_;
  Ogre::SceneNode *frame_node_;
  Ogre::ManualObject *manual_object_;
  Ogre::Entity *entity_;
  std::string material_name_;
};

} // namespace colorized_mesh_display

#endif // COLORIZED_MESH_VISUAL_HPP