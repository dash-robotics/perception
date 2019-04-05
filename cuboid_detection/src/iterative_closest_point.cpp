/*
 *********************************************************
 * *******************************************************
 * ***********  Iterative Closest Point (ICP) ************
 * *******************************************************
 * ************ written by Avinash on 04-02-2019 *********
 *********************************************************
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

static ros::Publisher icp_transform_pub;
std::string template_cuboid_filename;


void convert_icp_eigen_to_tf(Eigen::Matrix4f Tm)
{
    tf::Vector3 origin;
    static tf::TransformBroadcaster br;

    origin.setValue(static_cast<double>(Tm(0,3)),static_cast<double>(Tm(1,3)),static_cast<double>(Tm(2,3)));

    cerr << origin << endl;
    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(Tm(0,0)), static_cast<double>(Tm(0,1)), static_cast<double>(Tm(0,2)), 
        static_cast<double>(Tm(1,0)), static_cast<double>(Tm(1,1)), static_cast<double>(Tm(1,2)), 
        static_cast<double>(Tm(2,0)), static_cast<double>(Tm(2,1)), static_cast<double>(Tm(2,2)));

    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);

    tf::Transform transform;
    transform.setOrigin(origin);
    transform.setRotation(tfqt);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_frame", "cuboid_frame"));
}

void rawPCL_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr template_cuboid(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cuboid(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*input_cuboid);
   

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (template_cuboid_filename, *template_cuboid) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file");
        return;
    }
    
    pcl::IterativeClosestPoint <pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(input_cuboid);
    icp.setInputTarget(template_cuboid);
    pcl::PointCloud <pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    convert_icp_eigen_to_tf(icp.getFinalTransformation());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iterative_closest_point");
    ros::NodeHandle nh;

    nh.getParam("template_cuboid_path", template_cuboid_filename);
    cerr << "\nTemplate filename: " << template_cuboid_filename;
    
    ros::Subscriber raw_PCL_sub = nh.subscribe<sensor_msgs::PointCloud2>("/ground_plane_segmentation/points", 1, rawPCL_cb);
    
    ros::spin();
}
