/*
 *********************************************************
 * *******************************************************
 * ***** Get Three Dimension Bounding Box PointCloud *****
 * *******************************************************
 * ************ written by Avinash on 04-03-2019 *********
 *********************************************************
 */

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <opencv2/opencv.hpp>
#include <boost/foreach.hpp>
#include <color_object_detection/Rectangle.h>
using namespace std;


static vector<float> proj_matrix;
static bool obtain_camera_data;
static int64 two_dim_Bbox[4];
static ros::Publisher valid_inliers_pub;
static int64 min_inlier[2];
static int64 max_inlier[2];
static cv::Point3f valid_min_pt;
static cv::Point3f valid_max_pt;
static vector<cv::Point3f> valid_pts;


void check_inlier_projections(float x_det, float y_det, float z_det)
{
    float u = proj_matrix[0]*x_det + proj_matrix[1]*y_det + proj_matrix[2]*z_det + proj_matrix[3]*1;
    float v = proj_matrix[4]*x_det + proj_matrix[5]*y_det + proj_matrix[6]*z_det + proj_matrix[7]*1;
    float w = proj_matrix[8]*x_det + proj_matrix[9]*y_det + proj_matrix[10]*z_det + proj_matrix[11]*1;
    std::cout << "u = "<<u<<" v = "<<v<<" w = "<<w<<std::endl;

    float x = u/w;
    float y = v/w;

    if(two_dim_Bbox[0] < x && x < two_dim_Bbox[2])
    {
      if(two_dim_Bbox[1] < y && y < two_dim_Bbox[3])
      {  
        if(x < min_inlier[0] && y < min_inlier[1])
        {
          min_inlier[0] = x;
          min_inlier[1] = y;
          valid_min_pt.x = x_det;
          valid_min_pt.y = y_det;
          valid_min_pt.z = z_det;
        }
        else if(x > max_inlier[0] && y > max_inlier[1])
        { 
          max_inlier[0] = x;
          max_inlier[1] = y;
          valid_max_pt.x = x_det;
          valid_max_pt.y = y_det;
          valid_max_pt.z = z_det;
        }
      }
    }
}

void clear_variables()
{
  valid_pts.clear();
  proj_matrix.clear();
  obtain_camera_data = false;
}


/****************** Call Backs *********************/
/***** Start *****/
void cam_cb(const sensor_msgs::CameraInfoConstPtr& camInfo_msg)
{
  if(!obtain_camera_data)
  {
    std::cout << "Camera Info" << std::endl;
    for (int i=0;i<12;i++)
    {
      proj_matrix.push_back(camInfo_msg->P[i]);
      cout << proj_matrix[i];
    }
    
    obtain_camera_data = true;
  }
}


void bbox_cb(const color_object_detection::Rectangle msg)
{
  two_dim_Bbox[0] = msg.x1;
  two_dim_Bbox[1] = msg.y1;
  two_dim_Bbox[2] = msg.x2;
  two_dim_Bbox[3] = msg.y2;
  cout << "Bounding Box" << endl;
  for(int i=0;i<4;i++)
    cout << two_dim_Bbox[i] << " ";
  cout << endl;
}


void rawPCL_cb(const sensor_msgs::PointCloud2ConstPtr msg)
{
    sensor_msgs::PointCloud out_pointcloud;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, out_pointcloud);

    BOOST_FOREACH (const geometry_msgs::Point32 point,out_pointcloud.points)
    {
      check_inlier_projections(point.x, point.y, point.z);
      cout << "Point Clouds "<<point.x<<"\t"<<point.y<<"\t"<<point.z;
    }

    valid_pts.push_back(valid_min_pt);
    valid_pts.push_back(valid_max_pt);

    sensor_msgs::PointCloud out_cloud;
    for(int i=0;i<valid_pts.size();i++)
    {
      out_cloud.points[i].x = valid_pts[i].x;
      out_cloud.points[i].y = valid_pts[i].y;
      out_cloud.points[i].z = valid_pts[i].z;
    }

    sensor_msgs::PointCloud2 final_cloud;
    sensor_msgs::convertPointCloudToPointCloud2(out_cloud,final_cloud);
    valid_inliers_pub.publish(final_cloud);

    clear_variables();

}

/***** End *****/


int main(int argc, char **argv)
{
    obtain_camera_data = false;
    ros::init(argc,argv,"three_dimension_to_two_dimension");
    ros::NodeHandle nh;
    valid_inliers_pub = nh.advertise<sensor_msgs::PointCloud2>("/three_dimension_to_two_dimension/points", 1000);
    ros::Subscriber camera_sub = nh.subscribe("/camera/color/camera_info", 1,cam_cb);  
    ros::Subscriber bounding_box_sub = nh.subscribe("/bbox",1000,bbox_cb);
    ros::Subscriber raw_PCL_sub = nh.subscribe("/ground_plane_segmentation/points", 1000, rawPCL_cb); // MsgType <sensor_msgs::PointCloud2>
    
    ros::spinOnce();
    return 0;
}



























































































//
// {
//   // cv::Rect2i pt;

//   // pt.x = msg[0];
//   // pt.y = msg[1];
//   // two_dim_Bbox.push_back(pt);

//   // pt.x = msg[2];
//   // pt.y = msg[3];
//   // two_dim_Bbox.push_back(pt); 
// }


// printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
// BOOST_FOREACH (const pcl::PointXYZ& pt, msg->data)
// {
//   printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
//   Convert_3d_to_2d(pt.x, pt.y, pt.z);
// }



// for(int i = 0 ; i < out_cloud.points.size(); ++i){
//   geometry_msgs::Point32 point;




// int x1 = two_dim_Bbox.x;
// int x2 = x1+two_dim_Bbox.width;
// int y1 = two_dim_Bbox.y;
// int y2 = x1+two_dim_Bbox.height;

// if(x1 < x && x < x2)
// {
//   if(y1 < y && y < y2)
//     is_inlier = true;
// }

// if(x > two_dim_Bbox[0].x && x < two_dim_Bbox[1].x)
// {
//   if(y > two_dim_Bbox[0].y && y < two_dim_Bbox[1].y)
//     is_inlier = true;
// }



//sort(valid_pts.begin(),valid_pts.end());

// vector<geometry_msgs::Point32> final_pts;
// int l = valid3dPts.size()-1;

// final_pts.push_back({valid3dPts[0].x,valid3dPts[0].y,valid3dPts[0].z});
// final_pts.push_back({valid3dPts[l].x,valid3dPts[l].y,valid3dPts[l].z});

//bool is_inlier = false;
//is_inlier = true;
// if (is_inlier)
// {
//  
//   temp_pt.x = x_det;
//   temp_pt.y = y_det;
//   temp_pt.z = z_det;
//   valid_pts.push_back(temp_pt);
// }