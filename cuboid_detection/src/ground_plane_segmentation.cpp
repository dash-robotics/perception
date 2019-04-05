/*
Author: Heethesh Vhavle
Version: 1.0.1
Date: Apr 01, 2019

References:
http://wiki.ros.org/pcl/Tutorials
http://pointclouds.org/documentation/tutorials/extract_indices.php

CMake:
add_executable(ground_plane_segmentation src/ground_plane_segmentation.cpp)
target_link_libraries(ground_plane_segmentation ${catkin_LIBRARIES})
*/

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

// Publishers
ros::Publisher pcl_pub;
ros::Publisher coef_pub;

// Topics
bool invert;
double voxel_size;
double distance_threshold;
std::string input_topic;
std::string output_topic;
std::string coefficients_topic;

// Debug flag
bool debug = false;

void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PCLPointCloud2::Ptr cloud_ptr(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_filtered_ptr(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*input, *cloud_ptr);
    if (debug) std::cerr << "PointCloud before filtering: " << cloud_ptr->width << " " << cloud_ptr->height << " data points." << std::endl;

    // Filter the points in z-axis
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud(cloud_ptr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 0.9);
    pass.filter(*cloud_filtered_ptr);
    if (debug) std::cerr << "PointCloud after filtering: " << cloud_filtered_ptr->width << " " << cloud_filtered_ptr->height << " data points." << std::endl;

    // Downsample the points
    pcl::PCLPointCloud2::Ptr voxel_ptr(new pcl::PCLPointCloud2);
    pcl::VoxelGrid<pcl::PCLPointCloud2> downsample;
    downsample.setInputCloud(cloud_filtered_ptr);
    downsample.setLeafSize(voxel_size, voxel_size, voxel_size);
    downsample.filter(*voxel_ptr);

    // Setup ground plane segmentation
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Convert to the templated PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_voxel_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*voxel_ptr, *pcl_voxel_ptr);

    // Segmentation paramters    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(distance_threshold);

    // Segment the largest planar component from the cloud
    seg.setInputCloud(pcl_voxel_ptr);
    seg.segment(*inliers, *coefficients);

    // Extract the inliers
    pcl::PCLPointCloud2::Ptr plane_cloud_ptr(new pcl::PCLPointCloud2);
    pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
    extract.setInputCloud(voxel_ptr);
    extract.setIndices(inliers);
    extract.setNegative(invert);
    extract.filter(*plane_cloud_ptr);
    if (debug) std::cerr << "PointCloud representing the planar component: " << plane_cloud_ptr->width << " " << plane_cloud_ptr->height << " data points." << std::endl;

    // Publish the model coefficients
    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl_conversions::fromPCL(*coefficients, ros_coefficients);
    coef_pub.publish(ros_coefficients);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*plane_cloud_ptr, output);
    pcl_pub.publish(output);
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "ground_plane_segmentation");
    ros::NodeHandle nh("~");

    // Get params from launch file
    nh.getParam("invert", invert);
    nh.getParam("voxel_size", voxel_size);
    nh.getParam("distance_threshold", distance_threshold);
    nh.getParam("input", input_topic);
    nh.getParam("output", output_topic);
    nh.getParam("plane_coefficients", coefficients_topic);

    // Params defaults
    nh.param<bool>("invert", invert, true);
    nh.param<double>("voxel_size", voxel_size, 0.01);
    nh.param<double>("distance_threshold", distance_threshold, 0.01);
    nh.param<std::string>("input", input_topic, "/camera/depth/color/points");
    nh.param<std::string>("output", output_topic, "/ground_plane_segmentation/points");
    nh.param<std::string>("plane_coefficients", coefficients_topic, "/ground_plane_segmentation/coefficients");

    // Display params
    std::cout << "\nInvert Segmentation: " << invert << std::endl;
    std::cout << "Voxel Size: " << voxel_size << std::endl;
    std::cout << "Distance Threshold: " << distance_threshold << std::endl;
    std::cout << "Input Topic: " << input_topic << std::endl;
    std::cout << "Output Topic: " << output_topic << std::endl;
    std::cout << "Co-efficients Topic: " << coefficients_topic << std::endl;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe(input_topic, 1, callback);

    // Create a ROS publisher for the output segmented point cloud and coefficients
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 1);
    coef_pub = nh.advertise<pcl_msgs::ModelCoefficients>(coefficients_topic, 1);

    // Spin
    ros::spin();
}
