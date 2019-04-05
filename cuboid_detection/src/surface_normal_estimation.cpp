/*
Author: Sohil Savla
Version: 1.0.1
Date: Apr 02, 2019

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
ros::Publisher pcl_pub_top;
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

pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

void coefficients_callback(const pcl_msgs::ModelCoefficients& input)
{
    pcl_conversions::toPCL(input, *coefficients);
}

std::pair<pcl_msgs::ModelCoefficients, pcl::PCLPointCloud2::Ptr> getNormal(pcl::PCLPointCloud2::Ptr cloud_ptr, 
pcl::SacModel model, Eigen::Vector3f plane_normal, bool invert_local)
{
    // Setup ground plane segmentation
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Convert to the templated PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_voxel_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_ptr, *pcl_voxel_ptr);

    // Segmentation paramters    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(model);
    // seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(plane_normal);
    seg.setEpsAngle(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(distance_threshold);

     // Segment the largest planar component from the cloud
    seg.setInputCloud(pcl_voxel_ptr);
    seg.segment(*inliers, *coefficients);

    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl_conversions::fromPCL(*coefficients, ros_coefficients);

    // Extract the inliers
    pcl::PCLPointCloud2::Ptr plane_cloud_ptr(new pcl::PCLPointCloud2);
    pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(invert_local);
    extract.filter(*plane_cloud_ptr);

    std::pair<pcl_msgs::ModelCoefficients, pcl::PCLPointCloud2::Ptr> plane_info;
    plane_info = make_pair(ros_coefficients, plane_cloud_ptr);

    return plane_info;
}

void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // If Model Coefficients have been recieved:
    Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

    if (debug) ROS_INFO("Got here.");
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PCLPointCloud2::Ptr cloud_ptr(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_filtered_ptr(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*input, *cloud_ptr);
    if (debug) std::cerr << "PointCloud before filtering: " << cloud_ptr->width << " " << cloud_ptr->height << " data points." << std::endl;
    
    std::pair<pcl_msgs::ModelCoefficients, pcl::PCLPointCloud2::Ptr> plane1;
    pcl::PCLPointCloud2::Ptr plane_cloud_ptr_1(new pcl::PCLPointCloud2);
    plane1 = getNormal(cloud_ptr, pcl::SACMODEL_PERPENDICULAR_PLANE, plane_normal, invert);
    plane_cloud_ptr_1 = plane1.second;
    Eigen::Vector3f normal_z(plane1.first.values[0],plane1.first.values[1],plane1.first.values[2]);

    std::pair<pcl_msgs::ModelCoefficients, pcl::PCLPointCloud2::Ptr> plane2;
    pcl::PCLPointCloud2::Ptr plane_cloud_ptr_2(new pcl::PCLPointCloud2);
    plane2 = getNormal(plane_cloud_ptr_1, pcl::SACMODEL_PARALLEL_PLANE, plane_normal, invert);
    plane_cloud_ptr_2 = plane2.second;
    Eigen::Vector3f normal_y(plane2.first.values[0],plane2.first.values[1],plane2.first.values[2]);

    std::pair<pcl_msgs::ModelCoefficients, pcl::PCLPointCloud2::Ptr> plane3;
    pcl::PCLPointCloud2::Ptr plane_cloud_ptr_3(new pcl::PCLPointCloud2);
    plane3 = getNormal(plane_cloud_ptr_2, pcl::SACMODEL_PARALLEL_PLANE, plane_normal, !invert);
    plane_cloud_ptr_3 = plane3.second;
    Eigen::Vector3f normal_x(plane3.first.values[0],plane3.first.values[1],plane3.first.values[2]);

    
    std::cerr << "X Y Dot product: " << normal_x.dot(normal_y) << std::endl;
    std::cerr << "Y Z Dot product: " << normal_y.dot(normal_z) << std::endl;
    std::cerr << "Z X Dot product: " << normal_z.dot(normal_x) << std::endl;

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*plane_cloud_ptr_3, output);
    pcl_pub.publish(output);
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "surface_normal_estimation");
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
    nh.param<std::string>("input", input_topic, "/ground_plane_segmentation/points");
    nh.param<std::string>("output", output_topic, "/surface_segmentation/points");
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
    ros::Subscriber sub_coeff = nh.subscribe("/ground_plane_segmentation/coefficients", 1, coefficients_callback);

    // Create a ROS publisher for the output segmented point cloud and coefficients
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 1);
    pcl_pub_top = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 1);
    coef_pub = nh.advertise<pcl_msgs::ModelCoefficients>(coefficients_topic, 1);

    // Spin
    ros::spin();
}
