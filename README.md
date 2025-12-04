## polys_mapgen

`polys_mapgen` is a small ROS (catkin) package that generates a **3D point cloud map from convex polytopes (H‑polytopes)** and publishes it as a `sensor_msgs/PointCloud2` topic.  
It is designed to work together with planners such as `gcopter`, and comes with a launch file that starts RViz, the map generator, and a global planner node.

⚠️ **IMPORTANT:** This package is under frequent development and updates. Features and behaviors may change without notice. Use with caution and always check for the latest changes before deploying in production systems.

### Features
- **Convex polytope map representation**: obstacles are represented as half‑space constraints \(A x \le b\) (stored as `Eigen::MatrixX4d`).
- **Point cloud generation**: samples a 3D grid inside the map bounds and keeps only points that lie inside at least one polytope.
- **ROS integration**:
  - Publishes a `PointCloud2` map on `polys_pc_map` (remappable, e.g. to `/voxel_map`).
  - Configurable via ROS parameters / YAML file.

### Package structure
- **`src/hpolys_map.cpp`**: implementation of `hpolys_map::MapGenerator`, which generates the point cloud.
- **`src/mapgen.cpp`**: main node `hpolys_mapgen_node` that sets up ROS, loads parameters, defines simple example polytopes, and calls `MapGenerator`.
- **`include/hpolys_map.hpp`**: header declaring `hpolys_map::MapGenerator` and `hpolys_map::hpolys_mapgen_config`.
- **`cfg/mapgen.yaml`**: example configuration for map size and origin.
- **`launch/map_thengcopter.launch`**: starts RViz, `hpolys_mapgen_node`, and a `gcopter` global planning node.

### Dependencies
Declared in `package.xml` / `CMakeLists.txt`:
- **Build tool**: `catkin`
- **ROS**: `roscpp`, `std_msgs`, `sensor_msgs`
- **PCL / ROS**: `pcl_ros`, `pcl_conversions`
- **Math / linear algebra**: `Eigen3`

### Building
Assuming a ROS catkin workspace:

```bash
cd ~/catkin_ws/src
git clone <this-repo-url> polys_mapgen
cd ..
catkin_make
source devel/setup.bash   # or setup.zsh
```

If you see include errors for `hpolys_map.hpp`, ensure that the package is built with catkin (so that the `include` directory from `CMakeLists.txt` is used) and that you are including it as:

```cpp
#include "hpolys_map.hpp"
```

### Configuration
The node reads its parameters from the private namespace (`~`) and is typically configured via a YAML file.

Default example (`cfg/mapgen.yaml`):


You can create your own YAML file and adjust:
- **`map_size_*`**: dimensions of the sampling box (meters).
- **`map_origin_*`**: origin offset (meters).

### Running
After building and sourcing your workspace:

```bash
roslaunch polys_mapgen map_thengcopter.launch
```

This will:
- Start RViz with a preconfigured layout (from `gcopter`).
- Start `hpolys_mapgen_node`, loading parameters from `cfg/mapgen.yaml`.
- Publish a point cloud map topic (by default `polys_pc_map`, remapped in the launch file to `/voxel_map`).

You can also run the node directly:

```bash
rosrun polys_mapgen hpolys_mapgen_node
```

Make sure parameters are loaded (via `rosparam load` or a custom launch file) before expecting valid output.

### Custom polytopes
In `src/mapgen.cpp`, a simple example cube is defined and pushed into a `std::vector<Eigen::MatrixX4d>` that is passed into `MapGenerator` via `hpolys_map::hpolys_mapgen_config`.  
You can replace this with your own set of convex polytopes to generate more complex maps.

### License
This package is released under the **MIT License** (see `package.xml`).

