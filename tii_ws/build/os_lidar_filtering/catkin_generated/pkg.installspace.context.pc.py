# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "rosbag;roscpp;rospy;std_msgs;sensor_msgs;roslib;pcl_conversions;pcl_ros;pcl_msgs;dynamic_reconfigure".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-los_lidar_filtering".split(';') if "-los_lidar_filtering" != "" else []
PROJECT_NAME = "os_lidar_filtering"
PROJECT_SPACE_DIR = "/workspace/tii_ws/install"
PROJECT_VERSION = "0.0.0"
