/*
Author: Avinash
Version: 1.0.1
Date: Apr 03, 2019
*/

#include <iostream>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

using namespace std;

// Globals  
ros::Publisher pcl_pub;
Eigen::Matrix4d icp_transform;
string template_cuboid_filename;

// Flags
bool DEBUG = false;
bool ICP_SUCCESS = false;

void convert_icp_eigen_to_tf(Eigen::Matrix4d Tm)
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
    while (true) br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "cuboid_frame"));
}

void icp_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // Compute ICP only once
    if (ICP_SUCCESS)
    {
        return;
    }

    // Point cloud containers
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
    
    // Run ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(input_cuboid);
    icp.setInputTarget(template_cuboid);
    icp.align(*output_cloud);
    icp_transform = icp.getFinalTransformation().cast<double>();

    // Convert ICP results and broadcast TF
    if (icp.hasConverged())
    {
        cerr << "\nICP Score: " << icp.getFitnessScore() << endl;
        cerr << "ICP Transform:\n" << icp_transform << endl;

        ICP_SUCCESS = true;
        // convert_icp_eigen_to_tf(icp_transform);

        // Convert to ROS data type
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*output_cloud, output);
        pcl_pub.publish(output);
    }
}

int main(int argc, char **argv)
{
    // Init node
    ros::init(argc, argv, "iterative_closest_point");
    ros::NodeHandle nh;

    // Handle params
    nh.getParam("template_cuboid_path", template_cuboid_filename);
    cerr << "\nTemplate filename: " << template_cuboid_filename;
    
    // Subscribers
    ros::Subscriber pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/ground_plane_segmentation/points", 1, icp_callback);
    
    // Publishers
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/icp/aligned_points", 1);

    ros::spin();
}