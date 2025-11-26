#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "hpolys_map.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "polys_mapgen");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ROS_INFO("polys_mapgen node started.");
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("polys_pc_map", 1);
    // Initialize the point cloud
    sensor_msgs::PointCloud2       output;
    // Load configuration parameters
    hpolys_map::hpolys_mapgen_config config;
    config.nh_private = &nh_private;
    config.size_x = nh_private.param("map_size_x", 10.0);
    config.size_y = nh_private.param("map_size_y", 10.0);
    config.size_z = nh_private.param("map_size_z", 5.0);
    config.origin << nh_private.param("map_origin_x", 0.0),
                        nh_private.param("map_origin_y", 0.0),
                        nh_private.param("map_origin_z", 0.0);
    // Define some example convex polytopes (as 4D matrices)
    std::vector<Eigen::MatrixX4d> hPolys;
    // Example: a cube defined by 6 planes
    Eigen::MatrixX4d cube(6,4);
    cube <<  1, 0, 0, -2,   
            -1, 0, 0, 1,    
            0, 1, 0, -2,   
            0,-1, 0, 1,    
            0, 0, 1, -2, 
            0, 0,-1, 1; 
    hPolys.push_back(cube);
    config.hPolys_ptr = &hPolys;
    config.output_ptr = &output;
    // Create MapGenerator instance
    hpolys_map::MapGenerator mapGen;
    mapGen.config(config);
    mapGen.generatePointCloud();
    // Publish the generated point cloud
    //! @note publish loop
    ros::Rate loop_rate(1); // 1 Hz
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}