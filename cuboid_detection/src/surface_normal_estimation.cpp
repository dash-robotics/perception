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
#include <pcl/common/centroid.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// Publishers
ros::Publisher pcl_pub;
ros::Publisher pcl_pub_top;
ros::Publisher coef_pub;
ros::Publisher normal_x_pub;
ros::Publisher normal_y_pub;
ros::Publisher normal_z_pub;

// Topics
bool invert;
double voxel_size;
double distance_threshold;
std::string input_topic;
std::string output_topic;
std::string coefficients_topic;

// Debug flag
bool debug = false;

struct planeSegementationOutput
{
    pcl_msgs::ModelCoefficients plane_normal;
    pcl::PCLPointCloud2::Ptr leftover_pc;
    pcl::PCLPointCloud2::Ptr plane_pc;
    Eigen::Vector4f midpoint;
    int num_of_points;
};


pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

void convert_icp_eigen_to_tf(Eigen::Matrix4f Tm)
{   
    // Set translation
    tf::Vector3 origin;
    origin.setValue(Tm(0, 3), Tm(1, 3), Tm(2, 3));
    
    // Set rotation
    tf::Quaternion quaternion;
    tf::Matrix3x3 rotation_matrix;
    rotation_matrix.setValue(Tm(0, 0), Tm(0, 1), Tm(0, 2),  
                             Tm(1, 0), Tm(1, 1), Tm(1, 2),  
                             Tm(2, 0), Tm(2, 1), Tm(2, 2));
    rotation_matrix.getRotation(quaternion);

    // Make tf transform message
    tf::Transform transform;
    transform.setOrigin(origin);
    transform.setRotation(quaternion);

    // Broadcast the transforms
    tf::TransformBroadcaster br;
    std::cerr<<"Publishing TF"<<std::endl;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_frame", "cuboid_frame"));
}

void coefficients_callback(const pcl_msgs::ModelCoefficients& input)
{
    pcl_conversions::toPCL(input, *coefficients);
}

planeSegementationOutput getNormal(pcl::PCLPointCloud2::Ptr cloud_ptr, 
pcl::SacModel model, Eigen::Vector3f plane_normal, bool invert_local)
{
    planeSegementationOutput plane;

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
    extract.setNegative(!invert_local);
    extract.filter(*plane_cloud_ptr);

    pcl::PCLPointCloud2::Ptr not_plane_cloud_ptr(new pcl::PCLPointCloud2);
    pcl::ExtractIndices<pcl::PCLPointCloud2> extract_;
    extract_.setInputCloud(cloud_ptr);
    extract_.setIndices(inliers);
    extract_.setNegative(invert_local);
    extract_.filter(*not_plane_cloud_ptr);

    // Convert to the templated PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_xyz_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*plane_cloud_ptr, *plane_xyz_cloud_ptr);

    std::pair<pcl_msgs::ModelCoefficients, pcl::PCLPointCloud2::Ptr> plane_info;
    plane_info = make_pair(ros_coefficients, plane_cloud_ptr);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*plane_xyz_cloud_ptr, centroid);

    plane.plane_normal = ros_coefficients;
    plane.leftover_pc = not_plane_cloud_ptr;
    plane.plane_pc = plane_cloud_ptr;
    plane.midpoint = centroid;
    plane.num_of_points = (int) plane_xyz_cloud_ptr->points.size ();

    return plane;
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
    
    planeSegementationOutput planes[3];
    Eigen::Vector3f normals[3];

    for(int i = 0; i < 3; i++)
    {
        if(i==0)
            planes[i] = getNormal(cloud_ptr, pcl::SACMODEL_PERPENDICULAR_PLANE, plane_normal, invert);
        else
            planes[i] = getNormal(cloud_ptr, pcl::SACMODEL_PARALLEL_PLANE, plane_normal, invert);
        cloud_ptr = planes[i].leftover_pc;
        normals[i] << planes[i].plane_normal.values[0], planes[i].plane_normal.values[1], planes[i].plane_normal.values[2];
    }

    for(int i = 0; i< 3; i++)
    {
        for(int j = i; j< 3; j++)
        {
            if(planes[i].num_of_points < planes[j].num_of_points)
            {
                planeSegementationOutput temp_plane = planes[i];
                planes[i] = planes[j];
                planes[j] = temp_plane;
            }
        }
    }

    if(normals[2].dot(normals[1].cross(normals[0])) < 0)
    {
        normals[2] = -normals[2];
    }

    // Projection of midpoint of y,z on z.
    float proj = normals[0].dot(planes[0].midpoint.head<3>() - planes[1].midpoint.head<3>());
    Eigen::Vector3f centroid = planes[0].midpoint.head<3>() - (proj*normals[0]);

    Eigen::Matrix4f Rt;
    Rt << normals[2](0), normals[1](0), normals[0](0), centroid(0), 
            normals[2](1), normals[1](1), normals[0](1), centroid(1),
            normals[2](2), normals[1](2), normals[0](2), centroid(2),
            0,0,0,1;
    
    convert_icp_eigen_to_tf(Rt);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud_ptr, output);
    pcl_pub.publish(output);
    
    normal_x_pub.publish(planes[2].plane_normal);
    normal_y_pub.publish(planes[1].plane_normal);
    normal_z_pub.publish(planes[0].plane_normal);
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
    normal_x_pub = nh.advertise<pcl_msgs::ModelCoefficients>("/surface_segmentation/normal_x_coefficients", 1);
    normal_y_pub = nh.advertise<pcl_msgs::ModelCoefficients>("/surface_segmentation/normal_y_coefficients", 1);
    normal_z_pub = nh.advertise<pcl_msgs::ModelCoefficients>("/surface_segmentation/normal_z_coefficients", 1);

    // Spin
    ros::spin();
}
