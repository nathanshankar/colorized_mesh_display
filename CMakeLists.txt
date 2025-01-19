cmake_minimum_required(VERSION 3.8)
project(colorized_mesh_display)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pluginlib REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/colorized_mesh_display/colorized_mesh_display.h
  include/colorized_mesh_display/colorized_mesh_visual.h
)

# Source files
set(SOURCES
  src/colorized_mesh_display.cpp
  src/colorized_mesh_visual.cpp
)

# Target
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${MOC_FILES})

target_include_directories(colorized_mesh_display PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
include_directories(${Ogre_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
  ${Ogre_LIBRARIES}
  ${Ogre_RenderSystem_LIBRARIES}  # Ensure the render system library is linked
)


# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_default_plugins
  geometry_msgs
  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_msgs
  pluginlib
)

# Install plugin
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install plugin description file
install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(FILES icons/classes/ColorizedMesh.png
        DESTINATION share/${PROJECT_NAME}/icons/classes
)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
