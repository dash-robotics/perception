/*
Author: Heethesh Vhavle, Avinash
Version: 1.0.2
Date: Apr 03, 2019
*/

#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>

using namespace std;

// Globals  
ros::Publisher pcl_pub;
ros::Publisher bbox_pub;
ros::Publisher template_pub;
ros::Publisher pose_pub;

Eigen::Matrix4d icp_transform;
string template_cuboid_filename;
sensor_msgs::PointCloud2 bbox_msg;
sensor_msgs::PointCloud2 output_msg;
sensor_msgs::PointCloud2 template_msg;
tf::TransformListener *listener;
double dimensions[3];
geometry_msgs::Pose pose_msg;
Eigen::Affine3d pose_transform;

// Flags
bool DEBUG = false;
bool ICP_SUCCESS = false;
bool FIRST = true;
bool POSE_FLAG = false;

tf::Transform get_tf_from_stamped_tf(tf::StampedTransform sTf)
{
    // Construct a transform using elements of sTf
    tf::Transform tf(sTf.getBasis(), sTf.getOrigin());
    return tf;
}

void publish_pose(Eigen::Matrix4d H)
{
    // Set translation
    tf::Vector3 origin;
    origin.setValue(H(0, 3), H(1, 3), H(2, 3));
    
    // Set rotation
    tf::Quaternion quaternion;
    tf::Matrix3x3 rotation_matrix;
    rotation_matrix.setValue(H(0, 0), H(0, 1), H(0, 2),  
                             H(1, 0), H(1, 1), H(1, 2),  
                             H(2, 0), H(2, 1), H(2, 2));
    rotation_matrix.getRotation(quaternion);

    // Make tf transform message
    tf::Transform transform;
    transform.setOrigin(origin);
    transform.setRotation(quaternion);

    // Make pose message
    geometry_msgs::Pose p;
    p.position.x = H(0, 3);
    p.position.y = H(1, 3);
    p.position.z = H(2, 3);
    p.orientation.x = quaternion.x();
    p.orientation.y = quaternion.y();
    p.orientation.z = quaternion.z();
    p.orientation.w = quaternion.w();

    // Broadcast the transforms
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "icp_cuboid_frame"));
    pose_pub.publish(p);
}

void publish_bounding_box(Eigen::Matrix4d H)
{
    // Extract cuboid dimensions
    double l = dimensions[0];
    double w = dimensions[1];
    double h = dimensions[2];

    // Create a point cloud from the vertices
    pcl::PointCloud<pcl::PointXYZ> box_cloud;
    box_cloud.push_back(pcl::PointXYZ(-l/2, -w/2, -h/2));
    box_cloud.push_back(pcl::PointXYZ(-l/2, -w/2, h/2));
    box_cloud.push_back(pcl::PointXYZ(-l/2, w/2, -h/2));
    box_cloud.push_back(pcl::PointXYZ(-l/2, w/2, h/2));
    box_cloud.push_back(pcl::PointXYZ(l/2, -w/2, -h/2));
    box_cloud.push_back(pcl::PointXYZ(l/2, -w/2, h/2));
    box_cloud.push_back(pcl::PointXYZ(l/2, w/2, -h/2));
    box_cloud.push_back(pcl::PointXYZ(l/2, w/2, h/2));

    // Transform point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(box_cloud, *transformed_cloud, H.cast<float>());

    if (FIRST)
    {
        FIRST = false;
        cout << "Bounding Box Points:" << endl;
        for (int i = 0; i < transformed_cloud->size(); i++)
        {
            cout << "X: " << transformed_cloud->points[i].x << " | "
                 << "Y: " << transformed_cloud->points[i].y << " | "
                 << "Z: " << transformed_cloud->points[i].z << endl;
        }
    }

    // Convert to ROS data type and publish
    pcl::toROSMsg(*transformed_cloud, bbox_msg);
    bbox_msg.header.frame_id = "camera_depth_optical_frame";
    bbox_pub.publish(bbox_msg);
}

void pose_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    POSE_FLAG = true;
    tf::poseMsgToEigen(pose_msg, pose_transform);
}

void icp_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
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
    icp.setMaximumIterations(2000);
    icp.setTransformationEpsilon(1e-9);
    // icp.setMaxCorrespondenceDistance(0.05);
    // icp.setEuclideanFitnessEpsilon(1);
    icp.setRANSACOutlierRejectionThreshold(1.5);
    icp.align(*output_cloud);
    icp_transform = icp.getFinalTransformation().cast<double>().inverse();

    // Convert ICP results and broadcast TF
    if (icp.hasConverged())
    {
        // Display ICP results
        if (DEBUG || !ICP_SUCCESS)
        {
            cerr << "\nICP Score: " << icp.getFitnessScore() << endl;
            cerr << "ICP Transform:\n" << icp_transform << endl;
        }
        ICP_SUCCESS = true;

        // Convert to ROS data type
        pcl::toROSMsg(*output_cloud, output_msg);
        pcl::toROSMsg(*template_cuboid, template_msg);

        // Publish template point cloud
        template_msg.header.frame_id = "camera_depth_optical_frame";
        pcl_pub.publish(output_msg);
        template_pub.publish(template_msg);
        publish_bounding_box(icp_transform);
        publish_pose(icp_transform);
    }
}

int main(int argc, char **argv)
{
    // Init node
    ros::init(argc, argv, "iterative_closest_point");
    ros::NodeHandle nh("~");

    // Handle params
    nh.getParam("template_cuboid_path", template_cuboid_filename);
    nh.getParam("length", dimensions[0]);
    nh.getParam("width", dimensions[1]);
    nh.getParam("height", dimensions[2]);

    // Display params
    cerr << "\nTemplate filename: " << template_cuboid_filename << endl;
    cerr << "Length (m): " << dimensions[0] << endl;
    cerr << "Width (m): " << dimensions[1] << endl;
    cerr << "Height (m): " << dimensions[2] << endl;
    
    // Subscribers
    ros::Subscriber pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/ground_plane_segmentation/points", 1, icp_callback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose>("/surface_segmentation/pose", 1, pose_callback);

    // Publishers
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/icp/aligned_points", 1);
    bbox_pub = nh.advertise<sensor_msgs::PointCloud2>("/icp/bbox_points", 1);
    template_pub = nh.advertise<sensor_msgs::PointCloud2>("/icp/template", 1);
    pose_pub = nh.advertise<geometry_msgs::Pose>("/icp/pose", 1);

    // Spin
    ros::spin();
}
