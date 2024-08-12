#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <os_lidar_filtering/pointcloud_processor.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "os_lidar_filtering_node");
    ros::NodeHandle nh;

    PointCloudProcessor processor(nh);
    ros::spin();
    return 0;

    // std::string bag_file;
    // // nh.param("bag_file", bag_file, std::string("my_rosbag.bag"));
    // bag_file = "/home/tahoun/git/tii_lidar_filtering/tii_ws/src/pcl_open3d_test/data/LiDARFilteringAssignment.bag";

    // rosbag::Bag bag;
    // bag.open(bag_file, rosbag::bagmode::Read);

    // std::vector<std::string> topics;
    // topics.push_back(std::string( ));

    // rosbag::View view(bag, rosbag::TopicQuery(topics));

    // for (rosbag::MessageInstance const m : view) {
    //     sensor_msgs::PointCloud2ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
    //     if (cloud_msg != nullptr) {
    //         processor.processPointCloud(cloud_msg);
    //     }
    // }

    // bag.close();
    // return 0;

}
