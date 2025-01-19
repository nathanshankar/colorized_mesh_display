[![CI](https://github.com/marip8/colorized_mesh_display/actions/workflows/main.yml/badge.svg)](https://github.com/marip8/colorized_mesh_display/actions/workflows/main.yml)
[![license - Apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)

# Colorized Mesh Display

An RViz display plugin that subscribes to messages of type `pcl_msgs/msg/PolygonMesh` and creates a per-vertex colorized display of the mesh

## Use-case Examples
pcl_msgs/msg/PolygonMesh:
        # Separate header for the polygonal surface
        std_msgs/Header header
            builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
            string frame_id
        # Vertices of the mesh as a point cloud
        sensor_msgs/PointCloud2 cloud
            #
            std_msgs/Header header
                builtin_interfaces/Time stamp
                    int32 sec
                    uint32 nanosec
                string frame_id
            uint32 height
            uint32 width
            PointField[] fields
                uint8 INT8    = 1
                uint8 UINT8   = 2
                uint8 INT16   = 3
                uint8 UINT16  = 4
                uint8 INT32   = 5
                uint8 UINT32  = 6
                uint8 FLOAT32 = 7
                uint8 FLOAT64 = 8
                string name      #
                uint32 offset    #
                uint8  datatype  #
                uint32 count     #
            bool    is_bigendian #
            uint32  point_step   #
            uint32  row_step     #
            uint8[] data         #
            bool is_dense        #
        # List of polygons
        Vertices[] polygons
            uint32[] vertices


### Visualization of heat map data

![Heat map colorized mesh](docs/heat_map_colorized_mesh.png)

### Projection of color images onto a mesh

![Image projection colorized mesh](docs/image_projection_colorized_mesh.png)
