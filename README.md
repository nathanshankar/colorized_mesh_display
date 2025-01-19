[![CI](https://github.com/marip8/colorized_mesh_display/actions/workflows/main.yml/badge.svg)](https://github.com/marip8/colorized_mesh_display/actions/workflows/main.yml)
[![license - Apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)

# Colorized Mesh Display

An RViz display plugin that subscribes to messages of type `pcl_msgs/msg/PolygonMesh` and creates a per-vertex colorized display of the mesh

## Use-case Examples
```cpp
interface PolygonMesh {
    // Separate header for the polygonal surface
    Header header;
    
    interface Header {
        Time stamp;
        string frame_id;
        
        interface Time {
            int32 sec;
            uint32 nanosec;
        }
    }
    
    // Vertices of the mesh as a point cloud
    PointCloud2 cloud;
    
    interface PointCloud2 {
        Header header;
        uint32 height;
        uint32 width;
        PointField[] fields;
        bool is_bigendian;
        uint32 point_step;
        uint32 row_step;
        uint8[] data;
        bool is_dense;
        
        interface PointField {
            uint8 INT8 = 1;
            uint8 UINT8 = 2;
            uint8 INT16 = 3;
            uint8 UINT16 = 4;
            uint8 INT32 = 5;
            uint8 UINT32 = 6;
            uint8 FLOAT32 = 7;
            uint8 FLOAT64 = 8;
            string name;
            uint32 offset;
            uint8 datatype;
            uint32 count;
        }
    }
    
    // List of polygons
    Vertices[] polygons;
    
    interface Vertices {
        uint32[] vertices;
    }
}
```


### Visualization of heat map data

![Heat map colorized mesh](docs/heat_map_colorized_mesh.png)

### Projection of color images onto a mesh

![Image projection colorized mesh](docs/image_projection_colorized_mesh.png)
