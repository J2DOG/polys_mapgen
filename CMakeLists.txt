cmake_minimum_required(VERSION 3.0.2)
project(polys_mapgen)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin and ROS dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  std_msgs
  sensor_msgs
  roslib
)

## System dependencies
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

###################################
## catkin package declaration    ##
###################################
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pcl_conversions pcl_ros sensor_msgs
  DEPENDS EIGEN3 PCL
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}  # Ensure Eigen headers are found
  ${YAMLCPP_INCLUDE_DIRS} # Include yaml-cpp headers
)

# Build library
add_library(${PROJECT_NAME}
  src/hpolys_map.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  # No OpenMP needed
)

# Build executable node
add_executable(hpolys_mapgen_node src/mapgen.cpp)

target_link_libraries(hpolys_mapgen_node
  ${PROJECT_NAME}
  ${YAMLCPP_LIBRARIES} # Link yaml-cpp library
)