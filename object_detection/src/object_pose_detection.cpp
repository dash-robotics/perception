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

#include "object_detection/boost.h"
#include <visualization_msgs/Marker.h>

// TF includes
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// Pose includes
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>

// PCL specific includes
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/filters/extract_indices.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

// Local includes
#include "object_detection/ObjectDetection.h"

using namespace std;

// Publishers
ros::Publisher pcl_pub;
ros::Publisher icp_pub;
ros::Publisher coef_pub;
ros::Publisher bbox_pub;
ros::Publisher template_pub;
ros::Publisher pose_pub;
ros::Publisher marker_pub;

// Globals
bool invert;
double voxel_size;
double distance_threshold;
string input_topic;
string output_topic;
string coefficients_topic;

int argmin = -1;
Eigen::Matrix4d icp_transform;
vector<Eigen::Matrix4d> icp_transforms;
sensor_msgs::PointCloud2 bbox_msg;
sensor_msgs::PointCloud2 output_msg;
sensor_msgs::PointCloud2 template_msg;
tf::TransformListener* listener;
geometry_msgs::Pose pose_msg;
Eigen::Affine3d pose_transform;
double dimensions[] = { 0.02, 0.15, 0.02 };
double icp_fitness_score;
pcl::PCLPointCloud2::Ptr input_pcl(new pcl::PCLPointCloud2);

// Template paths
string template_path;
string template_filenames[] = { "", "screwdriver_ascii_tf.pcd", "eraser_ascii_tf.pcd",
    "clamp_ascii_tf.pcd", "marker_ascii_tf.pcd" };

// Flags
bool DEBUG = false;
bool ICP_SUCCESS = false;
bool FIRST = true;
bool POSE_FLAG = false;

void publish_grasp_marker(geometry_msgs::Pose& p)
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "camera_depth_optical_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker
    marker.ns = "grasp_pose";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = p.position.x;
    marker.pose.position.y = p.position.y;
    marker.pose.position.z = p.position.z;
    marker.pose.orientation.x = p.orientation.x;
    marker.pose.orientation.y = p.orientation.y;
    marker.pose.orientation.z = p.orientation.z;
    marker.pose.orientation.w = p.orientation.w;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = dimensions[0];
    marker.scale.y = dimensions[1];
    marker.scale.z = dimensions[2];

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5f;

    // Publish the marker
    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
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

    // Publish visualization marker
    publish_grasp_marker(p);

    // Broadcast the transforms
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "object_frame"));
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
    box_cloud.push_back(pcl::PointXYZ(-l / 2, -w / 2, -h / 2));
    box_cloud.push_back(pcl::PointXYZ(-l / 2, -w / 2, h / 2));
    box_cloud.push_back(pcl::PointXYZ(-l / 2, w / 2, -h / 2));
    box_cloud.push_back(pcl::PointXYZ(-l / 2, w / 2, h / 2));
    box_cloud.push_back(pcl::PointXYZ(l / 2, -w / 2, -h / 2));
    box_cloud.push_back(pcl::PointXYZ(l / 2, -w / 2, h / 2));
    box_cloud.push_back(pcl::PointXYZ(l / 2, w / 2, -h / 2));
    box_cloud.push_back(pcl::PointXYZ(l / 2, w / 2, h / 2));

    // Transform point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(box_cloud, *transformed_cloud, H.cast<float>());

    // cout << "Bounding Box Points:" << endl;
    // for (int i = 0; i < transformed_cloud->size(); i++)
    // {
    //     cout << "X: " << transformed_cloud->points[i].x << " | "
    //          << "Y: " << transformed_cloud->points[i].y << " | "
    //          << "Z: " << transformed_cloud->points[i].z << endl;
    // }

    // Convert to ROS data type and publish
    pcl::toROSMsg(*transformed_cloud, bbox_msg);
    bbox_msg.header.frame_id = "camera_depth_optical_frame";
    bbox_pub.publish(bbox_msg);
}

double icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl, pcl::PointCloud<pcl::PointXYZ>::Ptr template_pcl)
{
    int max_iter = 0;
    while (1) {
        // Point cloud containers
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Run ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(input_pcl);
        icp.setInputTarget(template_pcl);
        icp.setMaximumIterations(5000);
        icp.setTransformationEpsilon(1e-9);
        // icp.setMaxCorrespondenceDistance(0.05);
        icp.setEuclideanFitnessEpsilon(icp_fitness_score);
        icp.setRANSACOutlierRejectionThreshold(1.5);
        icp.align(*output_cloud);
        Eigen::Matrix4d icp_transform_local = icp.getFinalTransformation().cast<double>().inverse();
        icp_transforms.push_back(icp_transform_local);

        cerr << "ICP Score Before: " << icp.getFitnessScore() << endl;
        max_iter++;

        if ((icp.hasConverged() && icp.getFitnessScore() < icp_fitness_score) || (max_iter > 10)) {
            // Display ICP results
            cerr << "\nICP Score: " << icp.getFitnessScore() << endl;
            cerr << "ICP Transform:\n"
                 << icp_transform_local << endl;

            // Convert to ROS data type
            pcl::toROSMsg(*output_cloud, output_msg);
            pcl::toROSMsg(*template_pcl, template_msg);
            return icp.getFitnessScore();
        }
    }
}

void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Save the input point cloud to a global variable
    pcl_conversions::toPCL(*input, *input_pcl);
    if (DEBUG)
        cerr << "PointCloud before filtering: " << input_pcl->width << " " << input_pcl->height << " data points." << endl;

    // Publish template point cloud
    if (ICP_SUCCESS and argmin != -1) {
        icp_pub.publish(output_msg);
        template_msg.header.frame_id = "camera_depth_optical_frame";
        template_pub.publish(template_msg);
        // publish_bounding_box(icp_transform);
        publish_pose(icp_transform);
    }
}

bool service_callback(object_detection::ObjectDetection::Request& req,
    object_detection::ObjectDetection::Response& res)
{
    // Filter the points in z-axis
    pcl::PCLPointCloud2::Ptr cloud_filtered_ptr_z(new pcl::PCLPointCloud2);
    pcl::PassThrough<pcl::PCLPointCloud2> pass_z;
    pass_z.setInputCloud(input_pcl);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.0, 0.9);
    pass_z.filter(*cloud_filtered_ptr_z);
    if (DEBUG)
        cerr << "PointCloud after filtering: " << cloud_filtered_ptr_z->width << " " << cloud_filtered_ptr_z->height << " data points." << endl;

    // Filter the points in y-axis
    pcl::PCLPointCloud2::Ptr cloud_filtered_ptr(new pcl::PCLPointCloud2);
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud(cloud_filtered_ptr_z);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.2, 0.2);
    pass.filter(*cloud_filtered_ptr);
    if (DEBUG)
        cerr << "PointCloud after filtering in x-axis: " << cloud_filtered_ptr->width << " " << cloud_filtered_ptr->height << " data points." << endl;

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
    if (DEBUG)
        cerr << "PointCloud representing the planar component: " << plane_cloud_ptr->width << " " << plane_cloud_ptr->height << " data points." << endl;

    // Additional filtering
    pcl::PCLPointCloud2::Ptr cloud_filtered_ptr_z2(new pcl::PCLPointCloud2);
    pcl::PassThrough<pcl::PCLPointCloud2> pass_z2;
    pass_z2.setInputCloud(plane_cloud_ptr);
    pass_z2.setFilterFieldName("z");
    pass_z2.setFilterLimits(0.0, 0.75);
    pass_z2.filter(*cloud_filtered_ptr_z2);
    if (DEBUG)
        cerr << "PointCloud after filtering: " << cloud_filtered_ptr_z2->width << " " << cloud_filtered_ptr->height << " data points." << endl;

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud_filtered_ptr_z2, output);
    pcl_pub.publish(output);

    // Create the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_cleaned(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_filtered_ptr_z2, *pcl_cloud_cleaned);
    tree->setInputCloud(pcl_cloud_cleaned);

    // Create the extraction object for the clusters
    vector<pcl::PointIndices> object_cluster_indices;
    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // specify euclidean cluster parameters
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(200);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(pcl_cloud_cleaned);
    // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
    ec.extract(object_cluster_indices);

    // Display number of objects detceted
    cerr << "\nRequested Object: " << template_filenames[req.object_id] << endl;
    cerr << "Objects Detected: " << object_cluster_indices.size() << endl;

    // Find the best registered object
    int obj_id = req.object_id;
    vector<double> icp_scores;
    icp_transforms.clear();
    ICP_SUCCESS = false;

    for (vector<pcl::PointIndices>::const_iterator it = object_cluster_indices.begin(); it != object_cluster_indices.end(); ++it) {
        // Create a pcl object to hold the extracted cluster
        pcl::PCLPointCloud2::Ptr object_cluster(new pcl::PCLPointCloud2);
        pcl::ExtractIndices<pcl::PCLPointCloud2> obj_extract;
        obj_extract.setInputCloud(cloud_filtered_ptr_z2);
        obj_extract.setIndices(boost::make_shared<const pcl::PointIndices>(*it));
        obj_extract.setNegative(false);
        obj_extract.filter(*object_cluster);

        // Create a pcl object to hold the extracted cluster
        pcl::PointCloud<pcl::PointXYZ>::Ptr object_cluster_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*object_cluster, *object_cluster_pcl);
        // pcl::io::savePCDFile("/home/heethesh/ROS-Workspaces/dash_ws/src/perception/object_detection/templates/clamp.pcd", *object_cluster_pcl, true);

        // Convert to ROS data type
        // sensor_msgs::PointCloud2 output;
        // pcl_conversions::fromPCL(*object_cluster, output);
        // pcl_pub.publish(output);

        // Read template point cloud
        string template_filename = template_path + template_filenames[req.object_id];
        pcl::PointCloud<pcl::PointXYZ>::Ptr template_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(template_filename, *template_pcl) == -1) {
            PCL_ERROR("Couldn't read the template PCL file");
            res.success = false;
            return false;
        }

        // Register using ICP and broadcast TF
        double score = icp_registration(object_cluster_pcl, template_pcl);
        icp_scores.push_back(score);
    }

    // Find the object with best ICP score
    double min_score = 1000;
    argmin = -1;
    for (int i = 0; i < icp_scores.size(); i++) {
        if (icp_scores[i] < min_score) {
            argmin = i;
            min_score = icp_scores[i];
        }
    }

    cerr << "\nLowest Object Index: " << argmin << " with Score: " << min_score << endl;
    icp_transform = icp_transforms[argmin];

    if (min_score < icp_fitness_score) {
        cerr << "ICP Registration Success!\n"
             << endl;
        ICP_SUCCESS = true;
        res.success = true;
        return true;
    } else {
        cerr << "ICP Registration Failed to Converge within Threshold!\n"
             << endl;
        ICP_SUCCESS = false;
        res.success = false;
        return false;
    }
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "object_pose_detection");
    ros::NodeHandle nh("~");

    // Get params from launch file
    nh.getParam("invert", invert);
    nh.getParam("voxel_size", voxel_size);
    nh.getParam("distance_threshold", distance_threshold);
    nh.getParam("input", input_topic);
    nh.getParam("output", output_topic);
    nh.getParam("icp_fitness_score", icp_fitness_score);
    nh.getParam("template_path", template_path);

    // Params defaults
    nh.param<bool>("invert", invert, true);
    nh.param<double>("voxel_size", voxel_size, 0.01);
    nh.param<double>("distance_threshold", distance_threshold, 0.01);
    nh.param<string>("input", input_topic, "/camera/depth/color/points");
    nh.param<string>("output", output_topic, "/object_pose_detection/points");

    // Display params
    cout << "\nInvert Segmentation: " << invert << endl;
    cout << "Voxel Size: " << voxel_size << endl;
    cout << "Distance Threshold: " << distance_threshold << endl;
    cout << "Input Topic: " << input_topic << endl;
    cout << "Output Topic: " << output_topic << endl;
    cerr << "ICP Fitness Score: " << icp_fitness_score << endl;
    cerr << "ICP Template Folder: " << template_path << endl;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe(input_topic, 1, pcl_callback);
    ros::ServiceServer service = nh.advertiseService("detect_objects", service_callback);

    // Create a ROS publisher for the output segmented point cloud and coefficients
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 1);
    icp_pub = nh.advertise<sensor_msgs::PointCloud2>("/icp/registered_pcl", 1);
    bbox_pub = nh.advertise<sensor_msgs::PointCloud2>("/icp/bbox_points", 1);
    template_pub = nh.advertise<sensor_msgs::PointCloud2>("/icp/template", 1);
    pose_pub = nh.advertise<geometry_msgs::Pose>("/icp/pose", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("object_pose_detection/grasp_pose", 1);

    // Spin
    ros::spin();
}

// int main(int argc, char **argv)
// {
//     // Init node
//     ros::init(argc, argv, "iterative_closest_point");
//     ros::NodeHandle nh("~");

//     template_pub = nh.advertise<sensor_msgs::PointCloud2>("/icp/template", 1);
//     string template_cuboid_filename = "/home/heethesh/ROS-Workspaces/dash_ws/src/perception/object_detection/templates/marker_ascii_tf.pcd";

//     while(1)
//     {
//         // Point cloud containers
//         pcl::PointCloud<pcl::PointXYZ>::Ptr template_input(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::PointCloud<pcl::PointXYZ>::Ptr template_cuboid(new pcl::PointCloud<pcl::PointXYZ>);

//         // Read template point cloud
//         if (pcl::io::loadPCDFile<pcl::PointXYZ>(template_cuboid_filename, *template_cuboid) == -1)
//         {
//             PCL_ERROR("Couldn't read the template PCL file");
//             return 0;
//         }

//         // Convert to ROS data type
//         pcl::toROSMsg(*template_cuboid, template_msg);

//         // Publish template point cloud
//         template_msg.header.frame_id = "cuboid_frame";
//         template_pub.publish(template_msg);

//         // Sleep
//         ros::Duration(0.5).sleep();
//     }

//     // Spin
//     ros::spin();
// }
