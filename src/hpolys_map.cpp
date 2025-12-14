# include "hpolys_map.hpp"

using namespace hpolys_map;

MapGenerator::MapGenerator()
{
    ROS_INFO("[polys_mapgen]:MapGenerator initialized.");
}

// Destructor
MapGenerator::~MapGenerator(){}

void MapGenerator::config(hpolys_mapgen_config& config)
{
    ROS_INFO("[polys_mapgen]:MapGenerator configuration set.");
    config_ = config;
}

void MapGenerator::generatePointCloud()
{
    ROS_INFO("[polys_mapgen]:Generating point cloud from convex polytopes...");
    // iterate all points in the defined map area
    // check if the point is inside any of the convex polytopes
    // if true, add to point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    float step_size = 0.25; // step size  (m)
    // point_now
    Eigen::Vector3d point_now;
    bool inside;
    Eigen::VectorXd Ab;

    for (double x = 0; x < config_.size_x; x += step_size)
    {
        for (double y = 0; y < config_.size_y; y += step_size)
        {
            for (double z = 0; z < config_.size_z; z += step_size)
            {
                point_now << x + config_.origin(0), y + config_.origin(1), z + config_.origin(2);
                // check if inside any polytope
                inside = false;
                for (const auto& poly : *(config_.hPolys_ptr))
                {
                    Ab = poly * point_now.homogeneous();
                    if ((Ab.array() <= 0).all())
                    {
                        inside = true;
                        break;
                    }
                }
                if (inside)
                {
                    cloud.points.emplace_back(pcl::PointXYZ(point_now(0), point_now(1), point_now(2)));
                } 
            }
        }
    }
    // optimize point cloud
    

    // save to output
    pcl::toROSMsg(cloud, *(config_.output_ptr));
    // print info of output point cloud
    
    config_.output_ptr->header.frame_id = "odom";
    //time stamp
    config_.output_ptr->header.stamp = ros::Time::now();
    ROS_INFO("[polys_mapgen]:Point cloud generation completed with %lu points.", cloud.points.size());
    // kill templeary variables
    //cloud.clear();
}

void MapGenerator::generateCubesPointCloud()
{
    
}


