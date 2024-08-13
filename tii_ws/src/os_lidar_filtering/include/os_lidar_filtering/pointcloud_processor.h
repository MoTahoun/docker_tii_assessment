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
#include <dynamic_reconfigure/server.h>
#include <os_lidar_filtering/FilterConfig.h>
#include <map>
#include <string>


class PointCloudProcessor {
private:
    ros::Subscriber point_cloud_sub_;
    ros::Publisher filtered_cloud_pub_;
    ros::Publisher removed_range_pub_;
    ros::Publisher removed_noise_pub_;
    ros::Publisher removed_plane_pub_;
    ros::Subscriber pointCloud_subscriber_;
    ros::Publisher filteredCloud_publisher_;
    ros::Publisher remobedNoise_publisher_;
    ros::WallTime start_;
    ros::WallTime end_;
    double execution_time;

    double min_range_;
    double max_range_;
    double noise_mean_k_;
    double noise_stddev_;
    double radius_search_;
    int min_neighbors_;
    int max_iterations_;
    double ground_distance_threshold_;

    dynamic_reconfigure::Server<os_lidar_filtering::FilterConfig> server_;
    dynamic_reconfigure::Server<os_lidar_filtering::FilterConfig>::CallbackType f_;

    void dynamicReconfigureCallback(os_lidar_filtering::FilterConfig &config, uint32_t level);

    // void preProcessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void filterByRange(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& removed);
    void filterNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr& removedNoise);
    void filterGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& removedNoise);

    // void filterGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
    //                        pcl::PointIndices::Ptr& ground_indices);

    void voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    // void concatenatePointClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1,
    //                             const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2,
    //                             pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud)
    
    // void filterGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
    //                        pcl::PointIndices::Ptr& ground_indices);

public:
    PointCloudProcessor(ros::NodeHandle& nh, const std::map<std::string, std::string>& params);
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};

#endif // OS_LIDAR_FILTERING