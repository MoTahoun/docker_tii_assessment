#include <os_lidar_filtering/pointcloud_processor.h>
#include <pcl_conversions/pcl_conversions.h>



PointCloudProcessor::PointCloudProcessor(ros::NodeHandle& nh, const std::map<std::string, std::string>& params) {
    ROS_INFO("Initializing PointCloudProcessor");

    // Get topic name
    std::string input_topic = params.at("input_topic");
    std::string output_filtered_topic = params.at("output_filtered_topic");
    std::string output_noise_topic = params.at("output_noise_topic");

    // Initialize Parameters & Dynamic configure server
    leaf_size_ = std::stof(params.at("leaf_size"));
    intensity_min_ = std::stof(params.at("intensity_min"));
    intensity_max_ = std::stof(params.at("intensity_max"));
    min_range_ = std::stof(params.at("min_range"));
    max_range_ = std::stof(params.at("max_range"));
    noise_mean_k_ = std::stoi(params.at("noise_mean_k"));
    noise_stddev_ = std::stof(params.at("noise_stddev"));
    radius_search_ = std::stof(params.at("radius_search"));
    min_neighbors_ = std::stoi(params.at("min_neighbors"));
    max_iterations_ = std::stoi(params.at("max_iterations"));
    ground_distance_threshold_ = std::stof(params.at("ground_distance_threshold"));

    pointCloud_subscriber_ = nh.subscribe(input_topic, 1, &PointCloudProcessor::pointCloudCallback, this);
    filteredCloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(output_filtered_topic, 1);
    remobedNoise_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(output_noise_topic, 1);

    f_ = boost::bind(&PointCloudProcessor::dynamicReconfigureCallback, this, _1, _2);
    server_.setCallback(f_);

    
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

    ROS_INFO_STREAM("############### Message Received ##############");

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

    execution_time = (end_ - start_).toNSec() * 1e-6;
    total_execution_time += execution_time;
    ROS_INFO_STREAM("Filtered Range Exectution time (ms): " << execution_time);

    // Satatistical Outlier Removal Filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredSorCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removedSorNoiseCloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    start_ = ros::WallTime::now();
    statisticalOutlierRemovalFilter(filteredRangeCloud, filteredSorCloud, removedSorNoiseCloud);
    end_ = ros::WallTime::now();

    execution_time = (end_ - start_).toNSec() * 1e-6;
    total_execution_time += execution_time;
    ROS_INFO_STREAM("SOR Exectution time (ms): " << execution_time);

    // Radius Outlier Removal Filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredRorCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removedRorNoiseCloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    start_ = ros::WallTime::now();
    radiousOutlierRemovalFilter(filteredSorCloud, filteredRorCloud, removedRorNoiseCloud);
    end_ = ros::WallTime::now();

    execution_time = (end_ - start_).toNSec() * 1e-6;
    total_execution_time += execution_time;
    ROS_INFO_STREAM("ROR Exectution time (ms): " << execution_time);

    // Ground Plane from the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredGroundCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removedGroundCloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    start_ = ros::WallTime::now();
    groundPlaneFilter(filteredRorCloud, filteredGroundCloud, removedGroundCloud);
    end_ = ros::WallTime::now();

    execution_time = (end_ - start_).toNSec() * 1e-6;
    total_execution_time += execution_time;
    ROS_INFO_STREAM("Ground Plane Removal Exectution time (ms): " << execution_time);

    ROS_INFO_STREAM("Total Execution Time (ms): " << total_execution_time << "\n");

    pcl::PointCloud<pcl::PointXYZ>::Ptr removedNoiseCloud(new pcl::PointCloud<pcl::PointXYZ>);
    *removedNoiseCloud = *removedRangeCloud + *removedSorNoiseCloud + *removedRorNoiseCloud + *removedGroundCloud;

    sensor_msgs::PointCloud2 outputFiltered;
    pcl::toROSMsg(*filteredGroundCloud, outputFiltered);
    outputFiltered.header = cloud_msg->header;
    outputFiltered.fields = cloud_msg->fields;
    filteredCloud_publisher_.publish(outputFiltered);

    sensor_msgs::PointCloud2 outputNoise;
    pcl::toROSMsg(*removedNoiseCloud, outputNoise);
    outputNoise.header = cloud_msg->header;
    outputNoise.fields = cloud_msg->fields;
    remobedNoise_publisher_.publish(outputNoise);
}

void PointCloudProcessor::rangeFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& removed) {

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

void PointCloudProcessor::statisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                          pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                                                          pcl::PointCloud<pcl::PointXYZ>::Ptr& removedNoise) {
    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(noise_mean_k_);
    sor.setStddevMulThresh(noise_stddev_);

    // Filter inliers
    sor.filter(*filteredCloud);

    // Extract outliers
    sor.setNegative(true);
    sor.filter(*removedNoise);
}

void PointCloudProcessor::radiousOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& removedNoise) {

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(radius_search_);
    ror.setMinNeighborsInRadius(min_neighbors_);

    // Filter inliers
    ror.filter(*filteredCloud);

    // Extract outliers
    ror.setNegative(true);
    ror.filter(*removedNoise);
}

void PointCloudProcessor::groundPlaneFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
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

void PointCloudProcessor::voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    // Perform point cloud preprocessing here
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(0.1f, 0.1f, 0.1f);
    voxelGrid.filter(*cloud);
}

void PointCloudProcessor::passThroughFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr& filteredCloud,
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr& removedNoise) {
    
    pcl::PassThrough<pcl::PointXYZI> passIntensity;
    passIntensity.setInputCloud(cloud);
    passIntensity.setFilterFieldName("intensity");
    passIntensity.setFilterLimits(0.0, 255.0);
	
    passIntensity.filter(*removedNoise);

    passIntensity.setNegative(true);
    passIntensity.filter(*filteredCloud);
}

