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
#include "object_detection/boost.h"

// TF includes
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

// Pose includes
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// Publishers
ros::Publisher pcl_pub;
ros::Publisher coef_pub;
ros::Publisher bbox_pub;
ros::Publisher template_pub;
ros::Publisher pose_pub;

// Globals
bool invert;
double voxel_size;
double distance_threshold;
std::string input_topic;
std::string output_topic;
std::string coefficients_topic;

Eigen::Matrix4d icp_transform;
string template_cuboid_filename;
sensor_msgs::PointCloud2 bbox_msg;
sensor_msgs::PointCloud2 output_msg;
sensor_msgs::PointCloud2 template_msg;
tf::TransformListener *listener;
geometry_msgs::Pose pose_msg;
Eigen::Affine3d pose_transform;
double dimensions[3];
double icp_fitness_score;

// Flags
bool DEBUG = false;
bool ICP_SUCCESS = false;
bool FIRST = true;
bool POSE_FLAG = false;

void icp_registration(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // Compute ICP only once
    if (ICP_SUCCESS)
    {
        pcl_pub.publish(output_msg);
        template_msg.header.frame_id = "camera_depth_optical_frame";
        template_pub.publish(template_msg);
        publish_bounding_box(icp_transform);
        publish_pose(icp_transform);
        return;
    }

    // Point cloud containers
    pcl::PointCloud<pcl::PointXYZ>::Ptr template_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr template_cuboid(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cuboid(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Read input point cloud
    pcl::fromROSMsg(*msg, *input_cuboid);
    
    // Read template point cloud
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(template_cuboid_filename, *template_cuboid) == -1)
    {
        PCL_ERROR("Couldn't read the template PCL file");
        return;
    }

    // Transform the template point cloud to the estimated frame
    // if (!POSE_FLAG) return;
    // pcl::transformPointCloud(*template_input, *template_cuboid, pose_transform.cast<float>());

    // Run ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(input_cuboid);
    icp.setInputTarget(template_cuboid);
    icp.setMaximumIterations(5000);
    icp.setTransformationEpsilon(1e-9);
    // icp.setMaxCorrespondenceDistance(0.05);
    icp.setEuclideanFitnessEpsilon(icp_fitness_score);
    icp.setRANSACOutlierRejectionThreshold(1.5);
    icp.align(*output_cloud);
    icp_transform = icp.getFinalTransformation().cast<double>().inverse();

    // Convert ICP results and broadcast TF
    if (icp.hasConverged() && icp.getFitnessScore() < icp_fitness_score)
    {
        cerr << "\nICP Score: " << icp.getFitnessScore() << endl;
        // Display ICP results
        if (DEBUG || !ICP_SUCCESS)
        {
            cerr << "ICP Transform:\n" << icp_transform << endl;
            ICP_SUCCESS = true;
        }

        // Convert to ROS data type
        pcl::toROSMsg(*output_cloud, output_msg);
        pcl::toROSMsg(*template_cuboid, template_msg);

        // Publish template point cloud
        pcl_pub.publish(output_msg);
        template_msg.header.frame_id = "camera_depth_optical_frame";
        template_pub.publish(template_msg);
        publish_bounding_box(icp_transform);
        publish_pose(icp_transform);
    }
}

void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PCLPointCloud2::Ptr cloud_ptr(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_filtered_ptr_z(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_filtered_ptr(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*input, *cloud_ptr);
    if (DEBUG) std::cerr << "PointCloud before filtering: " << cloud_ptr->width << " " << cloud_ptr->height << " data points." << std::endl;

    // Filter the points in z-axis
    pcl::PassThrough<pcl::PCLPointCloud2> pass_z;
    pass_z.setInputCloud(cloud_ptr);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.0, 0.9);
    pass_z.filter(*cloud_filtered_ptr_z);
    if (DEBUG) std::cerr << "PointCloud after filtering: " << cloud_filtered_ptr_z->width << " " << cloud_filtered_ptr->height << " data points." << std::endl;

    // Filter the points in y-axis
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud(cloud_filtered_ptr_z);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.2, 0.2);
    pass.filter(*cloud_filtered_ptr);
    if (DEBUG) std::cerr << "PointCloud after filtering in x-axis: " << cloud_filtered_ptr->width << " " << cloud_filtered_ptr->height << " data points." << std::endl;

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
    if (DEBUG) std::cerr << "PointCloud representing the planar component: " << plane_cloud_ptr->width << " " << plane_cloud_ptr->height << " data points." << std::endl;

    // Additional filtering
    pcl::PCLPointCloud2::Ptr cloud_filtered_ptr_z2(new pcl::PCLPointCloud2);
    pcl::PassThrough<pcl::PCLPointCloud2> pass_z2;
    pass_z2.setInputCloud(plane_cloud_ptr);
    pass_z2.setFilterFieldName("z");
    pass_z2.setFilterLimits(0.0, 0.7);
    pass_z2.filter(*cloud_filtered_ptr_z2);
    if (DEBUG) std::cerr << "PointCloud after filtering: " << cloud_filtered_ptr_z2->width << " " << cloud_filtered_ptr->height << " data points." << std::endl;

    // Create the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_cleaned(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_filtered_ptr_z2, *pcl_cloud_cleaned);
    tree->setInputCloud(pcl_cloud_cleaned);

    // Create the extraction object for the clusters
    std::vector<pcl::PointIndices> object_cluster_indices;
    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // specify euclidean cluster parameters
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(pcl_cloud_cleaned);
    // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
    ec.extract(object_cluster_indices);

    // Display number of objects detceted
    std::cerr << "Objects Detected: " << object_cluster_indices.size() << std::endl;

    // What is this?
    for (std::vector<pcl::PointIndices>::const_iterator it = object_cluster_indices.begin(); it != object_cluster_indices.end(); ++it)
    {
        // Create a pcl object to hold the extracted cluster
        pcl::PCLPointCloud2::Ptr object_cluster(new pcl::PCLPointCloud2);
        pcl::ExtractIndices<pcl::PCLPointCloud2> obj_extract;
        obj_extract.setInputCloud(cloud_filtered_ptr_z2);
        obj_extract.setIndices(boost::make_shared<const pcl::PointIndices> (*it));
        obj_extract.setNegative(false);
        obj_extract.filter(*object_cluster);
        
        // Convert to ROS data type
        sensor_msgs::PointCloud2 output;
        pcl_conversions::fromPCL(*object_cluster, output);
        pcl_pub.publish(output);
        
        break;

        // now we are in a vector of indices pertaining to a single cluster.
        // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
        // for (std::vector<int>::const_iterator idx = it->indices.begin(); idx != it->indices.end(); ++idx)
        // {
        //     object_cluster->points.push_back(cloud_filtered_ptr_z2->points[*idx]);
        // }


        // // convert to pcl::PCLPointCloud2
        // pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);

        // // Convert to ROS data type
        // pcl_conversions::fromPCL(outputPCL, output);

        // // add the cluster to the array message
        // //clusterData.cluster = output;
        // CloudClusters.clusters.push_back(output);
    }

    // Publish the model coefficients
    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl_conversions::fromPCL(*coefficients, ros_coefficients);
    coef_pub.publish(ros_coefficients);
}

// int main(int argc, char** argv)
// {
//     // Initialize ROS
//     ros::init(argc, argv, "ground_plane_segmentation");
//     ros::NodeHandle nh("~");

//     // Get params from launch file
//     nh.getParam("invert", invert);
//     nh.getParam("voxel_size", voxel_size);
//     nh.getParam("distance_threshold", distance_threshold);
//     nh.getParam("input", input_topic);
//     nh.getParam("output", output_topic);
//     nh.getParam("plane_coefficients", coefficients_topic);

//     // Params defaults
//     nh.param<bool>("invert", invert, true);
//     nh.param<double>("voxel_size", voxel_size, 0.01);
//     nh.param<double>("distance_threshold", distance_threshold, 0.01);
//     nh.param<std::string>("input", input_topic, "/camera/depth/color/points");
//     nh.param<std::string>("output", output_topic, "/ground_plane_segmentation/points");
//     nh.param<std::string>("plane_coefficients", coefficients_topic, "/ground_plane_segmentation/coefficients");

//     // Display params
//     std::cout << "\nInvert Segmentation: " << invert << std::endl;
//     std::cout << "Voxel Size: " << voxel_size << std::endl;
//     std::cout << "Distance Threshold: " << distance_threshold << std::endl;
//     std::cout << "Input Topic: " << input_topic << std::endl;
//     std::cout << "Output Topic: " << output_topic << std::endl;
//     std::cout << "Co-efficients Topic: " << coefficients_topic << std::endl;

//     // Create a ROS subscriber for the input point cloud
//     ros::Subscriber sub = nh.subscribe(input_topic, 1, callback);

//     // Create a ROS publisher for the output segmented point cloud and coefficients
//     pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 1);
//     coef_pub = nh.advertise<pcl_msgs::ModelCoefficients>(coefficients_topic, 1);

//     // Spin
//     ros::spin();
// }

int main(int argc, char **argv)
{
    // Init node
    ros::init(argc, argv, "iterative_closest_point");
    ros::NodeHandle nh("~");

    template_pub = nh.advertise<sensor_msgs::PointCloud2>("/icp/template", 1);
    std::string template_cuboid_filename = "";

    while(1)
    {
        // Point cloud containers
        pcl::PointCloud<pcl::PointXYZ>::Ptr template_input(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr template_cuboid(new pcl::PointCloud<pcl::PointXYZ>);

        // Read template point cloud
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(template_cuboid_filename, *template_cuboid) == -1)
        {
            PCL_ERROR("Couldn't read the template PCL file");
            return;
        }

        // Convert to ROS data type
        pcl::toROSMsg(*template_cuboid, template_msg);
        
        // Publish template point cloud
        template_msg.header.frame_id = "camera_depth_optical_frame";
        template_pub.publish(template_msg);

        // Sleep
        ros::Duration(0.5).sleep();
    }

    // Spin
    ros::spin();
}