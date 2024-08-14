#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <os_lidar_filtering/pointcloud_processor.h>

/**
 * @brief function prototype
 *  This function is responsible for retrieving parameters from the ROS parameter server and storing them in a std::map. 
 */
void parseParameters(ros::NodeHandle& nh, std::map<std::string, std::string>& params);

int main(int argc, char** argv) {
    ros::init(argc, argv, "os_lidar_filtering_node");
    ros::NodeHandle nh("~");

    // Initializ Parameters Dictionary
    std::map<std::string, std::string> params;

    // Parse parameters from config/params.yaml file
    parseParameters(nh, params);

    PointCloudProcessor processor(nh, params);
    ros::spin();
    return 0;
}

/**
 * @brief This function is responsible for retrieving parameters from the ROS parameter server and storing them in a std::map. 
 * 
 * @param nh ros::NodeHandle
 * @param params reference to the std::map
 */
void parseParameters(ros::NodeHandle& nh, std::map<std::string, std::string>& params) {
    // Retrieve parameters from the parameter server
    nh.getParam("input_topic", params["input_topic"]);
    nh.getParam("output_filtered_topic", params["output_filtered_topic"]);
    nh.getParam("output_noise_topic", params["output_noise_topic"]);

    nh.getParam("range_output_filtered_topic", params["range_output_filtered_topic"]);
    nh.getParam("range_output_noise_topic", params["range_output_noise_topic"]);

    nh.getParam("sor_output_filtered_topic", params["sor_output_filtered_topic"]);
    nh.getParam("sor_output_noise_topic", params["sor_output_noise_topic"]);

    nh.getParam("ror_output_filtered_topic", params["ror_output_filtered_topic"]);
    nh.getParam("ror_output_noise_topic", params["ror_output_noise_topic"]);

    nh.getParam("plane_output_filtered_topic", params["plane_output_filtered_topic"]);
    nh.getParam("plane_output_noise_topic", params["plane_output_noise_topic"]);

    nh.getParam("clustering_output_filtered_topic", params["clustering_output_filtered_topic"]);
    nh.getParam("clustering_output_noise_topic", params["clustering_output_noise_topic"]);

    double leaf_size;
    nh.getParam("leaf_size", leaf_size);
    params["leaf_size"] = std::to_string(leaf_size);

    double min_intensity;
    nh.getParam("min_intensity", min_intensity);
    params["min_intensity"] = std::to_string(min_intensity);
    
    double max_intensity;
    nh.getParam("max_intensity", max_intensity);
    params["max_intensity"] = std::to_string(max_intensity);
    
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

    double cluster_tolerance;
    nh.getParam("cluster_tolerance", cluster_tolerance);
    params["cluster_tolerance"] = std::to_string(cluster_tolerance);
    
    int cluster_min;
    nh.getParam("cluster_min", cluster_min);
    params["cluster_min"] = std::to_string(cluster_min);

    int cluster_max;
    nh.getParam("cluster_max", cluster_max);
    params["cluster_max"] = std::to_string(cluster_max);
}