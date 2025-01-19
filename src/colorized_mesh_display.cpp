#include "colorized_mesh_display/colorized_mesh_display.h"
#include "colorized_mesh_display/colorized_mesh_visual.h"
#include <rviz_common/frame_manager_iface.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/logging.hpp>

namespace colorized_mesh_display
{

ColorizedMeshDisplay::ColorizedMeshDisplay()
{
}

ColorizedMeshDisplay::~ColorizedMeshDisplay()
{
}

void ColorizedMeshDisplay::onInitialize()
{
  MFDClass::onInitialize();
}

void ColorizedMeshDisplay::reset()
{
  MFDClass::reset();
  visual_.reset();
}

void ColorizedMeshDisplay::processMessage(const pcl_msgs::msg::PolygonMesh::ConstSharedPtr msg)
{
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;

  // rclcpp::Time msg_time(msg->header.stamp.sec, msg->header.stamp.nanosec);
  // if (!context_->getFrameManager()->getTransform(
  //         msg->header.frame_id, msg_time, position, orientation))
  // {
  //   RCLCPP_DEBUG(rclcpp::get_logger("ColorizedMeshDisplay"), "Error transforming into frame '%s'", msg->header.frame_id.c_str());
  //   return;
  // }

  pcl::PolygonMesh mesh;
  pcl_conversions::toPCL(*msg, mesh);

  visual_ = std::make_shared<ColorizedMeshVisual>(scene_manager_);
  visual_->visualizeMesh(mesh);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);
}

} // namespace colorized_mesh_display

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(colorized_mesh_display::ColorizedMeshDisplay, rviz_common::Display)