#ifndef HPOLYS_MAPGEN_HPP
#define HPOLYS_MAPGEN_HPP

#include <ros/ros.h>

#include <Eigen/Dense>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>


namespace
hpolys_map
{
struct hpolys_mapgen_config
{
    ros::NodeHandle *nh_private;
    // map size (m)
    double size_x;
    double size_y;
    double size_z;
    // offset (m)
    Eigen::Vector3f origin;
    // convex polytope obstacles 4d vectors [A|-b]
    std::vector<Eigen::MatrixX4d> *hPolys_ptr;
    // point cloud output
    sensor_msgs::PointCloud2 *output_ptr;
    /* data */
};

class MapGenerator
{
private:
    hpolys_mapgen_config config_;
public:
    MapGenerator();
    ~MapGenerator();
    void config(hpolys_mapgen_config& config);
    void generatePointCloud();
    void generateCubesPointCloud();
};


} // namespace hpolys_mapgen
#endif // HPOLYS_MAPGEN_HPP