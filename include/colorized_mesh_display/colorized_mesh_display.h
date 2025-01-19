#ifndef COLORIZED_MESH_DISPLAY_HPP
#define COLORIZED_MESH_DISPLAY_HPP

#include <rviz_common/message_filter_display.hpp>
#include <pcl_msgs/msg/polygon_mesh.hpp>
#include <memory>

namespace Ogre
{
class SceneNode;
}

namespace colorized_mesh_display
{

class ColorizedMeshVisual;

class ColorizedMeshDisplay : public rviz_common::MessageFilterDisplay<pcl_msgs::msg::PolygonMesh>
{
  Q_OBJECT

public:
  ColorizedMeshDisplay();
  ~ColorizedMeshDisplay() override;

protected:
  void onInitialize() override;
  void reset() override;

private:
  void processMessage(const pcl_msgs::msg::PolygonMesh::ConstSharedPtr msg) override;
  std::shared_ptr<ColorizedMeshVisual> visual_;
};

} // namespace colorized_mesh_display

#endif // COLORIZED_MESH_DISPLAY_HPP