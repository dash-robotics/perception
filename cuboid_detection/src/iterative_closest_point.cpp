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
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
static ros::Publisher icp_PCL_pub;

pcl::PointCloud<pcl::PointXYZ>::Ptr template_cuboid(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr input_cuboid(new pcl::PointCloud<pcl::PointXYZ>);

int performICP()
{
    pcl::IterativeClosestPoint <pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(input_cuboid);
    icp.setInputTarget(template_cuboid);
    pcl::PointCloud <pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    //icp_PCL_pub.publish(icp.getFinalTransformation());

    return (0);
}

void rawPCL_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*input_cuboid);
    performICP();
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"iterative_closest_point");
    ros::NodeHandle nh;
    //icp_PCL_pub = nh.advertise<geometry_msgs::>("iterative_closest_point", 1000);
    ros::Subscriber raw_PCL_sub = nh.subscribe<sensor_msgs::PointCloud2>("/ground_plane_segmentation/points", 1000, rawPCL_cb);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("$/{workspaceFolder}/src/perception/cuboid_detection/templates/template_cuboid_L200_W100_H75.pcd", *template_cuboid) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    
    ros::spinOnce();
    return 0;
}
