## polys_mapgen

`polys_mapgen` is a small ROS (catkin) package that generates a **3D point cloud map from convex polytopes (H‑polytopes)** and publishes it as a `sensor_msgs/PointCloud2` topic, attached with a launch file that starts RViz, the map generator, and a global planner node.
It is designed to generate your custom structed environments for planner to vertify algrithms quickly.


### Quick start
1. **Prerequisites**: Install ROS (has been vertified on Noetic) and Eigen3 (and PCL, yaml-cpp if not pulled in by ROS).
2. **Create the catkin workspace** (if you don’t have one):
   ```bash
   mkdir -p ~/catkin_ws/src && cd ~/catkin_ws
   ```
3. **Clone the package** into `src`:
   ```bash
   cd ~/catkin_ws/src
   git clone <this-repo-url> polys_mapgen
   ```
4. **Build**:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```
5. **Launch** (RViz + map generator; use your actual launch file name if different):
   ```bash
   roslaunch polys_mapgen map_and_rviz.launch
   ```
   This will:
    - Start RViz with a preconfigured layout (from `gcopter`).
    - Start `hpolys_mapgen_node`, loading parameters from `cfg/mapgen.yaml`.
    - Publish a point cloud map topic (by default `polys_pc_map`, remapped in the launch file to `/voxel_map`).



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

Make sure parameters are loaded (via `rosparam load` or a custom launch file) before expecting valid output.

### Custom polytopes
In `src/mapgen.cpp`, a simple example cube is defined and pushed into a `std::vector<Eigen::MatrixX4d>` that is passed into `MapGenerator` via `hpolys_map::hpolys_mapgen_config`.  
You can replace this with your own set of convex polytopes to generate more complex maps.

### License
This package is released under the **MIT License** (see `package.xml`).

