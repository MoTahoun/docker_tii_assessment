#include <os_lidar_filtering/pointcloud_processor.h>
#include <pcl_conversions/pcl_conversions.h>

/**
 * @brief Constructor for the PointCloudProcessor class.
 * 
 * This constructor initializes the PointCloudProcessor by setting up subscribers and publishers
 * for point cloud data, initializing filtering parameters, and setting up a dynamic reconfigure server.
 * 
 * @param nh ROS NodeHandle for managing communication with the ROS system.
 * @param params A map of parameters containing configuration settings for the processor.
 */

PointCloudProcessor::PointCloudProcessor(ros::NodeHandle& nh, const std::map<std::string, std::string>& params) {
    ROS_INFO("Initializing PointCloudProcessor");

    // Retrieve topic names from the parameter map
    std::string input_topic = params.at("input_topic");
    std::string output_filtered_topic = params.at("output_filtered_topic");
    std::string output_noise_topic = params.at("output_noise_topic");

    std::string range_output_filtered_topic = params.at("range_output_filtered_topic");
    std::string range_output_noise_topic = params.at("range_output_noise_topic");

    std::string sor_output_filtered_topic = params.at("sor_output_filtered_topic");
    std::string sor_output_noise_topic = params.at("sor_output_noise_topic");

    std::string ror_output_filtered_topic = params.at("ror_output_filtered_topic");
    std::string ror_output_noise_topic = params.at("ror_output_noise_topic");

    std::string plane_output_filtered_topic = params.at("plane_output_filtered_topic");
    std::string plane_output_noise_topic = params.at("plane_output_noise_topic");

    std::string clustering_output_filtered_topic = params.at("clustering_output_filtered_topic");
    std::string clustering_output_noise_topic = params.at("clustering_output_noise_topic");


    // Initialize filtering parameters from the parameter map
    leaf_size_ = std::stof(params.at("leaf_size"));
    min_intensity_ = std::stof(params.at("min_intensity"));
    max_intensity_ = std::stof(params.at("max_intensity"));
    
    min_range_ = std::stof(params.at("min_range"));
    max_range_ = std::stof(params.at("max_range"));
    noise_mean_k_ = std::stoi(params.at("noise_mean_k"));
    noise_stddev_ = std::stof(params.at("noise_stddev"));
    radius_search_ = std::stof(params.at("radius_search"));
    min_neighbors_ = std::stoi(params.at("min_neighbors"));
    max_iterations_ = std::stoi(params.at("max_iterations"));
    ground_distance_threshold_ = std::stof(params.at("ground_distance_threshold"));
    cluster_tolerance_ = std::stof(params.at("cluster_tolerance"));
    cluster_min_ = std::stoi(params.at("cluster_min"));
    cluster_max_ = std::stoi(params.at("cluster_max"));

    // Set up the subscriber for the lidar's input point cloud data
    pointCloud_subscriber_ = nh.subscribe(input_topic, 1, &PointCloudProcessor::pointCloudCallback, this);

    // Set up publishers for filtered and noise of the final point cloud data
    filteredCloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(output_filtered_topic, 1);
    removedNoise_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(output_noise_topic, 1);

    // Set up publishers for filtered and noise of the intermediate point cloud data
    filteredRange_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(range_output_filtered_topic, 1);
    removedRange_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(range_output_noise_topic, 1);

    filteredSOR_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(sor_output_filtered_topic, 1);
    removedSOR_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(sor_output_noise_topic, 1);

    filteredROR_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(ror_output_filtered_topic, 1);
    removedROR_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(ror_output_noise_topic, 1);

    filteredGround_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(plane_output_filtered_topic, 1);
    removedGround_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(plane_output_noise_topic, 1);

    filteredClustering_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(clustering_output_filtered_topic, 1);
    removedClustering_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(clustering_output_noise_topic, 1);

    // Set up the dynamic reconfigure server with a callback function
    f_ = boost::bind(&PointCloudProcessor::dynamicReconfigureCallback, this, _1, _2);
    server_.setCallback(f_);

    
}

/**
* @brief Callback function for dynamic reconfiguration.
* @param config Configuration parameters.
* @param level Level of configuration.
*/
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

    cluster_tolerance_ = config.cluster_tolerance;
    cluster_min_ = config.cluster_min;
    cluster_max_ = config.cluster_max;
}

/**
 * @brief Callback function for processing incoming point cloud messages.
 * 
 * This function is triggered whenever a new point cloud message is received. It processes
 * the point cloud by applying various filters, measures execution times for each filtering
 * step, and publishes the filtered and removed noise clouds.
 * 
 * @param cloud_msg Pointer to the incoming point cloud message.
 */
void PointCloudProcessor::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {

    // Check if the received cloud message is empty
    if (!cloud_msg) {
        ROS_ERROR("Received an empty cloud!");
        return;
    }

    ROS_INFO_STREAM("############### Message Received ##############");

    // Initialize total execution time
    total_execution_time = 0;

    // Convert ROS PointCloud2 message to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Remove points that are out of range
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredRangeCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removedRangeCloud(new pcl::PointCloud<pcl::PointXYZ>);
    start_ = ros::WallTime::now();
    rangeFilter(cloud, filteredRangeCloud, removedRangeCloud);
    end_ = ros::WallTime::now();

    // Range Filter execution time calculation in milliseconds
    execution_time = (end_ - start_).toNSec() * 1e-6;
    total_execution_time += execution_time;
    ROS_INFO_STREAM("Filtered Range Exectution time (ms): " << execution_time);

    // Satatistical Outlier Removal Filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredSorCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removedSorNoiseCloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    start_ = ros::WallTime::now();
    statisticalOutlierRemovalFilter(filteredRangeCloud, filteredSorCloud, removedSorNoiseCloud);
    end_ = ros::WallTime::now();

    // SOR Filter execution time calculation in milliseconds
    execution_time = (end_ - start_).toNSec() * 1e-6;
    total_execution_time += execution_time;
    ROS_INFO_STREAM("SOR Exectution time (ms): " << execution_time);

    // Radius Outlier Removal Filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredRorCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removedRorNoiseCloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    start_ = ros::WallTime::now();
    radiousOutlierRemovalFilter(filteredSorCloud, filteredRorCloud, removedRorNoiseCloud);
    end_ = ros::WallTime::now();

    // ROR Filter execution time calculation in milliseconds
    execution_time = (end_ - start_).toNSec() * 1e-6;
    total_execution_time += execution_time;
    ROS_INFO_STREAM("ROR Exectution time (ms): " << execution_time);

    // Ground Plane from the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredGroundCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removedGroundCloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    start_ = ros::WallTime::now();
    groundPlaneFilter(filteredRorCloud, filteredGroundCloud, removedGroundCloud);
    end_ = ros::WallTime::now();

    // RANSAC plane segmentation execution time calculation in milliseconds
    execution_time = (end_ - start_).toNSec() * 1e-6;
    total_execution_time += execution_time;
    ROS_INFO_STREAM("Ground Plane Removal Exectution time (ms): " << execution_time);

    // Euclidean Cluster Extraction
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredClusteringCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removedClusteringCloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    start_ = ros::WallTime::now();
    clusteringFilter(filteredGroundCloud, filteredClusteringCloud, removedClusteringCloud);
    end_ = ros::WallTime::now();

    // Euclidean Cluster Extraction execution time calculation in milliseconds
    execution_time = (end_ - start_).toNSec() * 1e-6;
    total_execution_time += execution_time;
    ROS_INFO_STREAM("Euclidean Cluster Extraction Exectution time (ms): " << execution_time);

    // Stream the total execution time of each received message
    ROS_INFO_STREAM("Total Execution Time (ms): " << total_execution_time << "\n");

    // Combine filtered and removed noise clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removedNoiseCloud(new pcl::PointCloud<pcl::PointXYZ>);
    *filteredCloud = *filteredGroundCloud;
    *removedNoiseCloud = *removedRangeCloud + *removedSorNoiseCloud + *removedRorNoiseCloud + *removedGroundCloud + *removedClusteringCloud;    
    
    
    // Publish filtered and noise clouds for each filter step
    // Post Range Filter Publishers
    sensor_msgs::PointCloud2 outputRangeFiltered;
    pcl::toROSMsg(*filteredRangeCloud, outputRangeFiltered);
    outputRangeFiltered.header = cloud_msg->header;
    outputRangeFiltered.fields = cloud_msg->fields;
    filteredRange_publisher_.publish(outputRangeFiltered);

    sensor_msgs::PointCloud2 outputRangeNoise;
    pcl::toROSMsg(*removedRangeCloud, outputRangeNoise);
    outputRangeNoise.header = cloud_msg->header;
    outputRangeNoise.fields = cloud_msg->fields;
    removedRange_publisher_.publish(outputRangeNoise);
    
    // Post Statistical Outliers Removal Publishers
    sensor_msgs::PointCloud2 outputSORFiltered;
    pcl::toROSMsg(*filteredSorCloud, outputSORFiltered);
    outputSORFiltered.header = cloud_msg->header;
    outputSORFiltered.fields = cloud_msg->fields;
    filteredSOR_publisher_.publish(outputSORFiltered);

    sensor_msgs::PointCloud2 outputSORNoise;
    pcl::toROSMsg(*removedSorNoiseCloud, outputSORNoise);
    outputSORNoise.header = cloud_msg->header;
    outputSORNoise.fields = cloud_msg->fields;
    removedSOR_publisher_.publish(outputSORNoise);

    // Post Radius Outliers Removal Publishers
    sensor_msgs::PointCloud2 outputRORFiltered;
    pcl::toROSMsg(*filteredRorCloud, outputRORFiltered);
    outputRORFiltered.header = cloud_msg->header;
    outputRORFiltered.fields = cloud_msg->fields;
    filteredROR_publisher_.publish(outputRORFiltered);

    sensor_msgs::PointCloud2 outputRORNoise;
    pcl::toROSMsg(*removedRorNoiseCloud, outputRORNoise);
    outputRORNoise.header = cloud_msg->header;
    outputRORNoise.fields = cloud_msg->fields;
    removedROR_publisher_.publish(outputRORNoise);

    // Post Gournd Plane Removal Publishers
    sensor_msgs::PointCloud2 outputGroundFiltered;
    pcl::toROSMsg(*filteredGroundCloud, outputGroundFiltered);
    outputGroundFiltered.header = cloud_msg->header;
    outputGroundFiltered.fields = cloud_msg->fields;
    filteredGround_publisher_.publish(outputGroundFiltered);

    sensor_msgs::PointCloud2 outputGroundNoise;
    pcl::toROSMsg(*removedGroundCloud, outputGroundNoise);
    outputGroundNoise.header = cloud_msg->header;
    outputGroundNoise.fields = cloud_msg->fields;
    removedGround_publisher_.publish(outputGroundNoise);

    // Post Ecludian Clustering Extractioin Publishers
    sensor_msgs::PointCloud2 outputClusteringFiltered;
    pcl::toROSMsg(*filteredClusteringCloud, outputClusteringFiltered);
    outputClusteringFiltered.header = cloud_msg->header;
    outputClusteringFiltered.fields = cloud_msg->fields;
    filteredClustering_publisher_.publish(outputClusteringFiltered);
    
    sensor_msgs::PointCloud2 outputClusteringNoise;
    pcl::toROSMsg(*removedClusteringCloud, outputClusteringNoise);
    outputClusteringNoise.header = cloud_msg->header;
    outputClusteringNoise.fields = cloud_msg->fields;
    removedClustering_publisher_.publish(outputClusteringNoise);

    // Publishers for the Final Cloud output of the filtered and removed noise  
    sensor_msgs::PointCloud2 outputFiltered;
    pcl::toROSMsg(*filteredCloud, outputFiltered);
    outputFiltered.header = cloud_msg->header;
    outputFiltered.fields = cloud_msg->fields;
    filteredCloud_publisher_.publish(outputFiltered);

    sensor_msgs::PointCloud2 outputNoise;
    pcl::toROSMsg(*removedNoiseCloud, outputNoise);
    outputNoise.header = cloud_msg->header;
    outputNoise.fields = cloud_msg->fields;
    removedNoise_publisher_.publish(outputNoise);
}

/**
 * @brief Filters points in a point cloud based on their distance from the origin.
 * 
 * This function iterates through an input point cloud and separates points into two categories:
 * those within a specified range from the origin and those outside of it. Points within the range
 * are added to the filtered cloud, while points outside the range are added to the removed cloud.
 * 
 * @param cloud The input point cloud to be filtered.
 * @param filteredCloud The output point cloud containing points within the specified range.
 * @param removed The output point cloud containing points outside the specified range.
 */
void PointCloudProcessor::rangeFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& removed) {

    // Iterate through the input cloud and filter based on distance from the origin
    for (const auto& point : cloud->points)
    {
        // Calculate the Euclidean distance of the point from the origin
        double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

        // Check if the point is within the specified range
        if (distance >= min_range_ && distance <= max_range_) {
            filteredCloud->points.push_back(point);
        }
        else {
            // Add point to the removed cloud
            removed->points.push_back(point);
        }
    }

    // Set the dimensions and properties of the filtered cloud
    filteredCloud->width = filteredCloud->points.size();
    filteredCloud->height = 1; // Unordered point cloud
    filteredCloud->is_dense = true;

    // Set the dimensions and properties of the removed cloud
    removed->width = removed->points.size();
    removed->height = 1; // Unordered point cloud
    removed->is_dense = true;
}

/**
 * @brief Applies a statistical outlier removal filter to a point cloud.
 * 
 * This function uses the PCL Statistical Outlier Removal filter to identify and remove noisy
 * points from a point cloud. It separates the points into inliers (filtered points) and outliers
 * (noise points) based on the mean distance to neighboring points.
 * 
 * @param cloud The input point cloud to be filtered.
 * @param filteredCloud The output point cloud containing inliers.
 * @param removedNoise The output point cloud containing outliers (noise points).
 */
void PointCloudProcessor::statisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                          pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                                                          pcl::PointCloud<pcl::PointXYZ>::Ptr& removedNoise) {
    
    // Create a Statistical Outlier Removal filter object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // Set the input cloud for the filter
    sor.setInputCloud(cloud);   
    // Set the number of nearest neighbors to use for mean distance estimation
    sor.setMeanK(noise_mean_k_);
    // Set the standard deviation multiplier threshold
    sor.setStddevMulThresh(noise_stddev_);

    // Filter inliers (points that are not considered outliers)
    sor.filter(*filteredCloud);

    // Extract outliers (noise points)
    sor.setNegative(true);
    sor.filter(*removedNoise);
}

/**
 * @brief Applies a radius outlier removal filter to a point cloud.
 * 
 * This function uses the PCL Radius Outlier Removal filter to identify and remove isolated
 * points from a point cloud. It separates the points into inliers (filtered points) and outliers
 * (noise points) based on the number of neighbors within a specified radius.
 * 
 * @param cloud The input point cloud to be filtered.
 * @param filteredCloud The output point cloud containing inliers.
 * @param removedNoise The output point cloud containing outliers (noise points).
 */
void PointCloudProcessor::radiousOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& removedNoise) {

    // Create a Radius Outlier Removal filter object
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    // Set the input cloud for the filter
    ror.setInputCloud(cloud);
    // Set the radius for neighbor search
    ror.setRadiusSearch(radius_search_);
    // Filter inliers (points that have enough neighbors within the radius)
    ror.setMinNeighborsInRadius(min_neighbors_);

    // Filter inliers (points that have enough neighbors within the radius)
    ror.filter(*filteredCloud);

    // Extract outliers (noise points)
    ror.setNegative(true);
    ror.filter(*removedNoise);
} 

/**
 * @brief Applies a ground plane removal filter to a point cloud.
 * 
 * This function uses the PCL SACSegmentation and ExtractIndices filters to identify and remove
 * the ground plane from a point cloud. It separates the points into inliers (ground plane points)
 * and outliers (non-ground points) based on a planar model.
 * 
 * @param cloud The input point cloud to be filtered.
 * @param filteredCloud The output point cloud containing non-ground points.
 * @param removedNoise The output point cloud containing ground plane points.
 */
void PointCloudProcessor::groundPlaneFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr& removedNoise) {

    // Create a PointIndices object to hold the indices of the ground plane inliers
    pcl::PointIndices::Ptr inliersGround(new pcl::PointIndices);
    // Create a ModelCoefficients object to hold the coefficients of the ground plane model
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    // Create a SACSegmentation object for segmenting the ground plane
    pcl::SACSegmentation<pcl::PointXYZ> segPlan;
    // Enable optimization of model coefficients
    segPlan.setOptimizeCoefficients(true);
    // Set the model type to plane
    segPlan.setModelType(pcl::SACMODEL_PLANE);
    // Use RANSAC method for segmentation
    segPlan.setMethodType(pcl::SAC_RANSAC);
    // Set the maximum number of iterations for RANSAC
    segPlan.setMaxIterations(max_iterations_);
    // Set the distance threshold for inliers
    segPlan.setDistanceThreshold(ground_distance_threshold_);

    // Segment the ground plane from the input cloud
    segPlan.setInputCloud(cloud);
    segPlan.segment(*inliersGround, *coefficients);

    // Create an ExtractIndices object for extracting non-ground points
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliersGround);

    // Extract points that are not part of the ground plane
    extract.setNegative(true);
    extract.filter(*filteredCloud);

    // Extract ground plane points
    extract.setNegative(false);
    extract.filter(*removedNoise);
}

/**
 * @brief Applies a clustering filter to a point cloud to separate clusters from noise.
 * 
 * This function uses the PCL Euclidean Cluster Extraction algorithm to identify clusters
 * of points in a point cloud. It separates the points into clusters (filtered points) and
 * noise (points that do not belong to any cluster).
 * 
 * @param cloud The input point cloud to be filtered.
 * @param filteredCloud The output point cloud containing clustered points.
 * @param removedNoise The output point cloud containing noise points.
 */
void PointCloudProcessor::clusteringFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr& removedNoise) {

    // Create the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // Vector to store the indices of each cluster
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // Set the spatial cluster tolerance
    ec.setClusterTolerance(cluster_tolerance_); // 0.02 -- > 2cm

    // Set the minimum and the maximum cluster sizes
    ec.setMinClusterSize(cluster_min_);
    ec.setMaxClusterSize(cluster_max_);
    // Set the search method to KdTree
    ec.setSearchMethod(tree);
    // Set the input cloud for clustering
    ec.setInputCloud(cloud);

    // Extract the clusters
    ec.extract(cluster_indices);

    // Create new point clouds to store clustered points and noise points
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr noise_cloud(new pcl::PointCloud<pcl::PointXYZ>(*cloud));

    // Create an ExtractIndices object for extracting clustered points
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);

    // Iterate over each cluster's indices
    for (const auto& indices : cluster_indices)
    {
        // Convert indices to PointIndices and extract the cluster points
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices(indices));
        extract.setIndices(inliers);
        extract.setNegative(false);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*cluster);
        // Add the cluster to the filtered cloud
        *filteredCloud += *cluster;

        // Extract points not in the cluster (noise)
        extract.setNegative(true);
        extract.filter(*removedNoise);
    }
}

/**
 * @brief Applies a voxel grid filter to downsample a point cloud.
 * 
 * This function uses the PCL VoxelGrid filter to reduce the number of points in a point cloud
 * by approximating the points within a voxel to a single point. This helps in reducing the
 * computational load for subsequent processing steps.
 * 
 * @param cloud The input point cloud to be downsampled. The downsampled cloud is stored in the same variable.
 */
void PointCloudProcessor::voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
     // Create a VoxelGrid filter object
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;

    // Set the input cloud and the leaf size for downsampling
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(0.1f, 0.1f, 0.1f);
    voxelGrid.filter(*cloud);
}

/**
 * @brief Applies a pass-through filter to a point cloud based on intensity values.
 * 
 * This function uses the PCL PassThrough filter to separate points in a point cloud
 * based on their intensity values. It filters out points that fall within a specified
 * intensity range and separates them into two categories: those within the range
 * (removed noise) and those outside the range (filtered points).
 * 
 * @param cloud The input point cloud to be filtered.
 * @param filteredCloud The output point cloud containing points outside the specified intensity range.
 * @param removedNoise The output point cloud containing points within the specified intensity range.
 */
void PointCloudProcessor::passThroughFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr& filteredCloud,
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr& removedNoise) {
    
    // Create a PassThrough filter object for intensity filtering
    pcl::PassThrough<pcl::PointXYZI> passIntensity;
    passIntensity.setInputCloud(cloud);
    // Set the field to filter by (intensity)
    passIntensity.setFilterFieldName("intensity");
    passIntensity.setFilterLimits(min_intensity_, max_intensity_);
	
    // Filter points within the intensity range (considered noise)
    passIntensity.filter(*removedNoise);

    // Filter points outside the intensity range (considered filtered points)
    passIntensity.setNegative(true);
    passIntensity.filter(*filteredCloud);
}

