#ifndef OS_LIDAR_FILTERING_H
#define OS_LIDAR_FILTERING_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <dynamic_reconfigure/server.h>
#include <os_lidar_filtering/FilterConfig.h>
#include <map>
#include <string>

/**
 * @class PointCloudProcessor
 * @brief A class for processing point cloud data using various filtering techniques to filter noise and ground plane from lidar data.
 */
class PointCloudProcessor {
private:
    // ROS Subscriber and Publishers
    ros::Subscriber pointCloud_subscriber_; ///< Subscriber for incoming point cloud data.
    ros::Publisher filteredCloud_publisher_; ///< Publisher for filtered point cloud data.
    ros::Publisher removedNoise_publisher_; ///< Publisher for removed noise data.

    ros::Publisher filteredRange_publisher_; ///< Publisher for range-filtered point cloud data.
    ros::Publisher removedRange_publisher_; ///< Publisher for removed range data.

    ros::Publisher filteredSOR_publisher_; ///< Publisher for Statistical Outlier Removal filtered data.
    ros::Publisher removedSOR_publisher_; ///< Publisher for removed SOR noise data.

    ros::Publisher filteredROR_publisher_; ///< Publisher for Radius Outlier Removal filtered data.
    ros::Publisher removedROR_publisher_; ///< Publisher for removed ROR noise data.

    ros::Publisher filteredGround_publisher_; ///< Publisher for ground-filtered point cloud data.
    ros::Publisher removedGround_publisher_; ///< Publisher for removed ground data.

    ros::Publisher filteredClustering_publisher_; ///< Publisher for clustering-filtered point cloud data.
    ros::Publisher removedClustering_publisher_; ///< Publisher for removed clustering data.
    
    // Timing variables
    ros::WallTime start_; ///< Start time for execution measurement.
    ros::WallTime end_; ///< End time for execution measurement.
    double execution_time; ///< Execution time for processing.
    double total_execution_time; ///< Total execution time for all processing.

    // Filtering parameters
    double leaf_size_; ///< Leaf size for voxel grid filtering.
    double min_intensity_, max_intensity_; ///< Intensity range for filtering.
    double min_range_, max_range_; ///< Range for passthrough filtering.
    double noise_mean_k_, noise_stddev_; ///< Parameters for statistical outlier removal.
    double radius_search_; ///< Radius search parameter for radius outlier removal.
    int min_neighbors_; ///< Minimum neighbors for radius outlier removal.
    int max_iterations_; ///< Maximum iterations for ground plane segmentation.
    double ground_distance_threshold_; ///< Distance threshold for ground plane segmentation.

    double cluster_tolerance_; ///< Tolerance for clustering.
    int cluster_min_, cluster_max_; ///< Minimum and maximum cluster size.

    // Dynamic reconfigure server
    dynamic_reconfigure::Server<os_lidar_filtering::FilterConfig> server_; ///< Dynamic reconfigure server.
    dynamic_reconfigure::Server<os_lidar_filtering::FilterConfig>::CallbackType f_; ///< Callback type for dynamic reconfigure.

    /**
     * @brief Callback function for dynamic reconfiguration.
     * @param config Configuration parameters.
     * @param level Level of configuration.
     */
    void dynamicReconfigureCallback(os_lidar_filtering::FilterConfig &config, uint32_t level);

    /**
     * @brief Apply a range filter to the point cloud.
     * @param cloud Input point cloud.
     * @param filteredCloud Output filtered point cloud.
     * @param removed Output removed points.
     */
    void rangeFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                     pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr& removed);

    /**
     * @brief Apply a statistical outlier removal filter to the point cloud.
     * @param cloud Input point cloud.
     * @param filteredCloud Output filtered point cloud.
     * @param removedNoise Output removed noise points.
     */
    void statisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr& removedNoise);

    /**
     * @brief Apply a radius outlier removal filter to the point cloud.
     * @param cloud Input point cloud.
     * @param filteredCloud Output filtered point cloud.
     * @param removedNoise Output removed noise points.
     */
    void radiousOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& removedNoise);

    /**
     * @brief Apply a ground plane filter to the point cloud.
     * @param cloud Input point cloud.
     * @param filteredCloud Output filtered point cloud.
     * @param removedNoise Output removed noise points.
     */
    void groundPlaneFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& removedNoise);

    /**
     * @brief Apply a clustering filter to the point cloud.
     * @param cloud Input point cloud.
     * @param filteredCloud Output filtered point cloud.
     * @param removedNoise Output removed noise points.
     */
    void clusteringFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                          pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr& removedNoise);

    /**
     * @brief Apply a voxel grid filter to the point cloud.
     * @param cloud Input point cloud.
     */
    void voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    /**
     * @brief Apply a pass-through filter to the point cloud.
     * @param cloud Input point cloud.
     * @param filteredCloud Output filtered point cloud.
     * @param removedNoise Output removed noise points.
     */
    void passThroughFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                           pcl::PointCloud<pcl::PointXYZI>::Ptr& filteredCloud,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr& removedNoise);  

public:
    /**
     * @brief Constructor for the PointCloudProcessor class.
     * @param nh ROS NodeHandle for parameter and topic management.
     * @param params Map of parameters for configuring the processor.
     */
    PointCloudProcessor(ros::NodeHandle& nh, const std::map<std::string, std::string>& params);

    /**
     * @brief Callback function for processing incoming point cloud messages.
     * @param cloud_msg Pointer to the incoming point cloud message.
     */
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};

#endif // OS_LIDAR_FILTERING 