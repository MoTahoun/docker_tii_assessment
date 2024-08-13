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
    ros::Subscriber pointCloud_subscriber_;
    ros::Publisher filteredCloud_publisher_;
    ros::Publisher remobedNoise_publisher_;
    ros::WallTime start_;
    ros::WallTime end_;
    double execution_time;
    double total_execution_time;

    double leaf_size_;
    double intensity_min_, intensity_max_;
    double min_range_, max_range_;
    double noise_mean_k_, noise_stddev_;
    double radius_search_;
    int min_neighbors_;
    int max_iterations_;
    double ground_distance_threshold_;

    dynamic_reconfigure::Server<os_lidar_filtering::FilterConfig> server_;
    dynamic_reconfigure::Server<os_lidar_filtering::FilterConfig>::CallbackType f_;

    void dynamicReconfigureCallback(os_lidar_filtering::FilterConfig &config, uint32_t level);

    // void preProcessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void rangeFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& removed);

    void statisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr& removedNoise);

    void radiousOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& removedNoise);

    void groundPlaneFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& removedNoise);

    void voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    void passThroughFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                           pcl::PointCloud<pcl::PointXYZI>::Ptr& filteredCloud,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr& removedNoise);  

public:
    PointCloudProcessor(ros::NodeHandle& nh, const std::map<std::string, std::string>& params);
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};

#endif // OS_LIDAR_FILTERING