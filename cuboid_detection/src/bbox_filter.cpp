/*
Author: Avinash, Heethesh
Version: 1.0.2
Date: Apr 03, 2019
*/

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <color_object_detection/Rectangle.h>

using namespace std;

// Flags
bool DEBUG = false;
bool READ_INFO = true;

int bbox[4];
int min_inlier[2];
int max_inlier[2];

ros::Publisher pcl_pub;
vector<double> proj_matrix;

bool within_bbox(float x, float y, float z)
{
    // Check if projection matrix is read
    if (READ_INFO) return false;

    // 3D to 2D projection
    float u = (proj_matrix[0] * x) + (proj_matrix[1] * y) + (proj_matrix[2] * z) + proj_matrix[3];
    float v = (proj_matrix[4] * x) + (proj_matrix[5] * y) + (proj_matrix[6] * z) + proj_matrix[7];
    float w = (proj_matrix[8] * x) + (proj_matrix[9] * y) + (proj_matrix[10] * z) + proj_matrix[11];
    
    // Normalize the uv points
    u /= w;
    v /= w;

    // Check if these points are within the 2D bbox
    if (bbox[0] < u && u < bbox[2]) {
        if (bbox[1] < v && v < bbox[3]) {
            return true;
        }
    }
    return false;
}

/****************** Callbacks *********************/

void info_cb(const sensor_msgs::CameraInfoConstPtr& camInfo_msg)
{
    if (READ_INFO)
    {
        if (DEBUG) cerr << "\nCamera Info:" << endl;
        for (int i = 0; i < 12; i++)
        {
            if (DEBUG) cerr << camInfo_msg->P[i] << " ";
            proj_matrix.push_back(camInfo_msg->P[i]);
        }
        READ_INFO = false;
    }
}

void bbox_cb(const color_object_detection::Rectangle msg)
{
    bbox[0] = msg.x1;
    bbox[1] = msg.y1;
    bbox[2] = msg.x2;
    bbox[3] = msg.y2;
}

void pcl_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PCLPointCloud2::Ptr input_cloud_ptr(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::toPCL(*input, *input_cloud_ptr);
    pcl::fromPCLPointCloud2(*input_cloud_ptr, *pcl_cloud_ptr);

    // Store the inliers
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Check inliers for all points `in the PCL
    for (int i = 0; i < pcl_cloud_ptr->size(); i++)
    {
        if (within_bbox(pcl_cloud_ptr->points[i].x, pcl_cloud_ptr->points[i].y, pcl_cloud_ptr->points[i].z))
        {
            inliers->indices.push_back(i);
        }
    }

    // Extract the inliers
    pcl::PCLPointCloud2::Ptr output_ptr(new pcl::PCLPointCloud2);
    pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
    extract.setInputCloud(input_cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*output_ptr);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*output_ptr, output);
    pcl_pub.publish(output);
}

int main(int argc, char** argv)
{
    // Setup node
    ros::init(argc, argv, "bbox_filter");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber bbox_sub = nh.subscribe("/object_detection/bbox", 1, bbox_cb);
    ros::Subscriber info_sub = nh.subscribe("/camera/color/camera_info", 1, info_cb);
    ros::Subscriber pcl_sub = nh.subscribe("/ground_plane_segmentation/points", 1, pcl_cb);

    // Publisher
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/bbox_filter/points", 1);

    // Spin
    ros::spin();
}
