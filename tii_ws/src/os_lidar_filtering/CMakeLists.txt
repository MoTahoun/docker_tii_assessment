cmake_minimum_required(VERSION 3.0.2)
project(os_lidar_filtering)
set(CMAKE_CXX_STANDARD 14)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # open3d_conversions
  pcl_conversions
  pcl_msgs
  pcl_ros
  rosbag
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
#System Dependencies
find_package(Eigen3 REQUIRED) 
# find_package(Open3D REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)


################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/FilterConfig.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rosbag roscpp rospy std_msgs sensor_msgs roslib pcl_conversions pcl_ros pcl_msgs
  DEPENDS dynamic_reconfigure
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR}) 
# include_directories(${Open3D_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

add_executable(os_lidar_filtering_node src/main.cpp src/pointcloud_processor.cpp)
add_dependencies(os_lidar_filtering_node ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(os_lidar_filtering_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# target_include_directories(os_lidar_filtering_node ${PCL_INCLUDE_DIRS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


#############
## Testing ##
#############

