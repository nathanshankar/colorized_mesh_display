#include <pcl/io/vtk_lib_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <pcl_msgs/msg/polygon_mesh.hpp>
#include <random>

template<typename T>
bool get(const std::shared_ptr<rclcpp::Node> &node, const std::string &key, T &val)
{
  if (!node->get_parameter(key, val))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to get '%s' parameter", key.c_str());
    return false;
  }
  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("colorized_mesh_display_test_node");

  std::string path;
  node->declare_parameter<std::string>("mesh_file", "");
  if(!get(node, "mesh_file", path))
    return -1;

  std::string base_frame;
  node->declare_parameter<std::string>("base_frame", "");
  if(!get(node, "base_frame", base_frame))
    return -1;

  // Load the mesh into a ROS message
  pcl_msgs::msg::PolygonMesh msg;
  {
    pcl::PolygonMesh mesh;
    if(pcl::io::loadPolygonFile(path, mesh) < 0)
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to load mesh file from '%s'", path.c_str());
      return -1;
    }
    RCLCPP_INFO(node->get_logger(), "Successfully loaded mesh file");

    // Convert the mesh
    pcl_conversions::fromPCL(mesh, msg);
    msg.header.frame_id = base_frame;
    msg.header.stamp = node->now();
  }

  // Publish the mesh
  auto pub = node->create_publisher<pcl_msgs::msg::PolygonMesh>("colorized_mesh", 1);
  pub->publish(msg);
  RCLCPP_INFO(node->get_logger(), "Published colorized mesh file");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
