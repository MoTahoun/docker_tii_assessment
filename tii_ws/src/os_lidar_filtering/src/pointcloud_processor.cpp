#include <os_lidar_filtering/pointcloud_processor.h>
#include <pcl_conversions/pcl_conversions.h>



PointCloudProcessor::PointCloudProcessor(ros::NodeHandle& nh) {
    ROS_INFO("Initializing PointCloudProcessor");
    // Initialize subscribers and publishers
    point_cloud_sub_ = nh.subscribe("/mbuggy/os2/points", 1, &PointCloudProcessor::pointCloudCallback, this);
    filtered_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_pointcloud", 1);
    removed_range_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/removed_range", 1);
    removed_noise_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/removed_noise", 1);
    removed_plane_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/removed_plane", 1);

    f_ = boost::bind(&PointCloudProcessor::dynamicReconfigureCallback, this, _1, _2);
    server_.setCallback(f_);

    // Default parameter values
    min_range_ = 3.0;
    max_range_ = 300.0;
    noise_mean_k_ = 78;
    noise_stddev_ = 3.8;
    radius_search_ = 2.0;
    min_neighbors_ = 4;
    max_iterations_ = 1000;
    ground_distance_threshold_ = 0.1;
}

void PointCloudProcessor::dynamicReconfigureCallback(os_lidar_filtering::FilterConfig& config, 
                                                     uint32_t level) {
    min_range_ = config.min_range;
    max_range_ = config.max_range;
    noise_mean_k_ = config.noise_mean_k;
    noise_stddev_ = config.noise_stddev;
    radius_search_ = config.radius_search;
    min_neighbors_ = config.min_neighbors;
    max_iterations_ = config.max_iterations;
    ground_distance_threshold_ = config.ground_distance_threshold;
}

void PointCloudProcessor::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
 
    if (!cloud_msg) {
        ROS_ERROR("Received an empty cloud!");
        return;
    }

    // Convert ROS PointCloud2 message to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Remove points that are out of range
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredRange(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removedRange(new pcl::PointCloud<pcl::PointXYZ>);
    start_ = ros::WallTime::now();
    filterByRange(cloud, filteredRange, removedRange);
    end_ = ros::WallTime::now();

    execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("Filtered Range Exectution time (ms): " << execution_time);

    // Remove noise from the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredSorRor(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removedNoise(new pcl::PointCloud<pcl::PointXYZ>);
    
    start_ = ros::WallTime::now();
    filterNoise(filteredRange, filteredSorRor, removedNoise);
    end_ = ros::WallTime::now();

    execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("SOR and ROS Exectution time (ms): " << execution_time);

    // Ground Plane from the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredGround(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removedGround(new pcl::PointCloud<pcl::PointXYZ>);
    
    start_ = ros::WallTime::now();
    filterGroundPlane(filteredSorRor, filteredGround, removedGround);
    end_ = ros::WallTime::now();

    execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("Ground Plane Removal Exectution time (ms): " << execution_time);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*filteredGround, output);
    output.header = cloud_msg->header;
    filtered_cloud_pub_.publish(output);

    sensor_msgs::PointCloud2 removed_range_output;
    pcl::toROSMsg(*removedRange, removed_range_output);
    removed_range_output.header = cloud_msg->header;
    removed_range_pub_.publish(removed_range_output);

    sensor_msgs::PointCloud2 removed_noise_output;
    pcl::toROSMsg(*removedNoise, removed_noise_output);
    removed_noise_output.header = cloud_msg->header;
    removed_noise_pub_.publish(removed_noise_output);

    sensor_msgs::PointCloud2 removed_plane_output;
    pcl::toROSMsg(*removedGround, removed_plane_output);
    removed_plane_output.header = cloud_msg->header;
    removed_plane_pub_.publish(removed_plane_output);
}

void PointCloudProcessor::filterByRange(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr& removed) {

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

    // Iterate through the input cloud and filter based on distance from the origin
    for (const auto& point : cloud->points)
    {
        double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

        if (distance >= min_range_ && distance <= max_range_) {
            filteredCloud->points.push_back(point);
        }
        else {
            removed->points.push_back(point);
        }
    }

    filteredCloud->width = filteredCloud->points.size();
    filteredCloud->height = 1; // Unordered point cloud
    filteredCloud->is_dense = true;

    removed->width = removed->points.size();
    removed->height = 1; // Unordered point cloud
    removed->is_dense = true;
}

void PointCloudProcessor::filterNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& removedNoise) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredSor(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr removedSor(new pcl::PointCloud<pcl::PointXYZ>());
    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(noise_mean_k_);
    sor.setStddevMulThresh(noise_stddev_);

    // Filter inliers
    sor.filter(*filteredSor);

    // Extract outliers
    sor.setNegative(true);
    sor.filter(*removedSor);

    // RadiusOutlierRemoval filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredSorRor(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr removedRor(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(filteredSor);
    outrem.setRadiusSearch(radius_search_);
    outrem.setMinNeighborsInRadius(min_neighbors_);

    // Filter inliers
    outrem.filter(*filteredSorRor);

    // Extract outliers
    outrem.setNegative(true);
    outrem.filter(*removedRor);

    *filteredCloud = *filteredSorRor;
    *removedNoise = *removedSor + *removedRor;
}

void PointCloudProcessor::filterGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr& removedNoise) {

    pcl::PointIndices::Ptr inliersGround(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iterations_);
    seg.setDistanceThreshold(ground_distance_threshold_);

    seg.setInputCloud(cloud);
    seg.segment(*inliersGround, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliersGround);
    extract.setNegative(true);
    extract.filter(*filteredCloud);

    extract.setNegative(false);
    extract.filter(*removedNoise);
}



void PointCloudProcessor::voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // Perform point cloud preprocessing here
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(0.1f, 0.1f, 0.1f);
    voxelGrid.filter(*cloud);
}

void PointCloudProcessor::passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // Perform point cloud preprocessing here
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> passX;
    passX.setInputCloud(cloud);
    passX.setFilterFieldName("x");
    passX.setFilterLimits(3, 200);
	// passX.setNegative(true);
    passX.filter(*cloud);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl::PassThrough<pcl::PointXYZ> passY;
    // passY.setInputCloud(cloud);
    // passY.setFilterFieldName("y");
    // passY.setFilterLimits(0, 200);
	// // passX.setNegative(true);
    // passY.filter(*cloud_filtered_y);

    // concatenatePointClouds(cloud_filtered_x, cloud_filtered_y, cloud);

    // // Enable extraction of removed indices
    // pass.setKeepOrganized(true);
    // pass.setExtractRemovedIndices(true);

    // // Apply the filter
    // pcl::PointIndices::Ptr removed_indices(new pcl::PointIndices);
    // pass.filter(*cloud_filtered);
    // pass.getRemovedIndices(*removed_indices);

    // // Use removed_indices to access the removed points from the original cloud
    // for (int index : removed_indices->indices) {
    //     pcl::PointXYZ removed_point = cloud->points[index];
    //     // Process the removed point
    // }
}





// void PointCloudProcessor::concatenatePointClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1,
//                                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2,
//                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud)
// {
//     // Concatenate the two point clouds
//     pcl::concatenatePointCloud(*cloud1, *cloud2, *output_cloud);
// }





// void PointCloudProcessor::preProcessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
//     // Define the distance range for filtering
//     double min_distance = 3.0;  // Minimum distance in meters
//     double max_distance = 200.0; // Maximum distance in meters

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

//     // Iterate through the input cloud and filter based on distance from the origin
//     for (const auto& point : cloud->points)
//     {
//         double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

//         if (distance >= min_distance && distance <= max_distance)
//         {
//             cloud_filtered->points.push_back(point);
//         }
//     }

//     cloud_filtered->width = cloud_filtered->points.size();
//     cloud_filtered->height = 1; // Unordered point cloud
//     cloud_filtered->is_dense = true;

//     *cloud = *cloud_filtered;

// }