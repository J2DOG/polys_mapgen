#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h> 
#include "hpolys_map.hpp"


std::vector<Eigen::MatrixX4d> loadObstaclesFromYAML(const std::string& filepath) {
    std::vector<Eigen::MatrixX4d> hPolys;
        YAML::Node config = YAML::LoadFile(filepath);
        if (!config["hpolys"]) {
            ROS_WARN("[load_obstacles]:No 'hpolys' key found in YAML file: %s", filepath.c_str());
            return hPolys;
        }
        for (const auto& polytope_node : config["hpolys"]) {
            int num_constraints = polytope_node.size();
            Eigen::MatrixX4d H(num_constraints, 4);
            for (int i = 0; i < num_constraints; ++i) {
                const auto& row = polytope_node[i];
                H(i, 0) = row[0].as<double>();
                H(i, 1) = row[1].as<double>();
                H(i, 2) = row[2].as<double>();
                H(i, 3) = row[3].as<double>(); 
            }
            hPolys.push_back(H);
        }
    ROS_INFO("[load_obstacles]:Loaded %lu obstacles from YAML.", hPolys.size());
    return hPolys;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "polys_mapgen");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ROS_INFO("[polys_mapgen]:polys_mapgen node started.");
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("polys_pc_map", 1);
    // Initialize the point cloud
    sensor_msgs::PointCloud2       output;
    // Load configuration parameters
    hpolys_map::hpolys_mapgen_config config;
    config.nh_private = &nh_private;
    config.size_x = nh_private.param("map_size_x", 10.0);
    config.size_y = nh_private.param("map_size_y", 10.0);
    config.size_z = nh_private.param("map_size_z", 5.0);
    config.origin << nh_private.param("map_origin_x", -5.0),
                        nh_private.param("map_origin_y", -5.0),
                        nh_private.param("map_origin_z", 0.0);
    // Define some example convex polytopes (as 4D matrices)
    std::vector<Eigen::MatrixX4d> hPolys;
    std::string pkg_path = ros::package::getPath("polys_mapgen");
    std::string obs_yaml_path = pkg_path + "/cfg/obstacles.yaml";
    hPolys = loadObstaclesFromYAML(obs_yaml_path);
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