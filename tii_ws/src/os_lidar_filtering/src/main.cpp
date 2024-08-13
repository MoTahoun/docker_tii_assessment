#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <os_lidar_filtering/pointcloud_processor.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "os_lidar_filtering_node");
    ros::NodeHandle nh("~");

    // Initializ Parameters Dictionary
    std::map<std::string, std::string> params;

    // Retrieve parameters from the parameter server
    nh.getParam("input_topic", params["input_topic"]);
    nh.getParam("output_filtered_topic", params["output_filtered_topic"]);
    nh.getParam("output_noise_topic", params["output_noise_topic"]);

    double min_range;
    nh.getParam("min_range", min_range);
    params["min_range"] = std::to_string(min_range);

    double max_range;
    nh.getParam("max_range", max_range);
    params["max_range"] = std::to_string(max_range);

    int noise_mean_k;
    nh.getParam("noise_mean_k", noise_mean_k);
    params["noise_mean_k"] = std::to_string(noise_mean_k);

    double noise_stddev;
    nh.getParam("noise_stddev", noise_stddev);
    params["noise_stddev"] = std::to_string(noise_stddev);

    double radius_search;
    nh.getParam("radius_search", radius_search);
    params["radius_search"] = std::to_string(radius_search);

    int min_neighbors;
    nh.getParam("min_neighbors", min_neighbors);
    params["min_neighbors"] = std::to_string(min_neighbors);

    int max_iterations;
    nh.getParam("max_iterations", max_iterations);
    params["max_iterations"] = std::to_string(max_iterations);

    double ground_distance_threshold;
    nh.getParam("ground_distance_threshold", ground_distance_threshold);
    params["ground_distance_threshold"] = std::to_string(ground_distance_threshold);


    PointCloudProcessor processor(nh, params);
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
