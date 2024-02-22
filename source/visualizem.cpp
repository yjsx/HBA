#include <string>
#include <stdio.h>
#include <fstream>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "ros/ros.h"
#include <math.h>
#include <rosbag/bag.h>
#include <ceres/ceres.h>

#include "ba.hpp"
#include "tools.hpp"
#include "mypcl.hpp"
#include "submap.hpp"

using namespace std;
using namespace Eigen;
typedef pcl::PointXYZRGB PointType2;

void displayProgressBar(int progress, int total) {
    const int barWidth = 50; // Width of the progress bar
    float percentage = (float)progress / total;
    int pos = barWidth * percentage;

    std::cout << "[";
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(percentage * 100.0) << " %\r";
    std::cout.flush(); // Ensure the progress is displayed
}
std::pair<int, int> get_submap_index(std::vector<Submap> submaps, int j){
	
	for(int i = 0; i < submaps.size(); i++){
		if(j < submaps[i].length)
			return std::make_pair(i, j);
		else	
			j -= submaps[i].length;
	}
}
void transform_pointcloud_rgb(pcl::PointCloud<PointType2> const& pc_in,
                            pcl::PointCloud<PointType2>& pt_out,
                            Eigen::Vector3d t,
                            Eigen::Quaterniond q)
  {
    size_t size = pc_in.points.size();
    pt_out.points.resize(size);
    for(size_t i = 0; i < size; i++)
    {
      Eigen::Vector3d pt_cur(pc_in.points[i].x, pc_in.points[i].y, pc_in.points[i].z);
      Eigen::Vector3d pt_to;
      // if(pt_cur.norm()<0.3) continue;
      pt_to = q * pt_cur + t;
      pt_out.points[i].x = pt_to.x();
      pt_out.points[i].y = pt_to.y();
      pt_out.points[i].z = pt_to.z();
      // pt_out.points[i].r = pc_in.points[i].r;
      // pt_out.points[i].g = pc_in.points[i].g;
      // pt_out.points[i].b = pc_in.points[i].b;
    }
  }
int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualize");
  ros::NodeHandle nh("~");

  ros::Publisher pub_map = nh.advertise<sensor_msgs::PointCloud2>("/cloud_map", 100);
  ros::Publisher pub_debug = nh.advertise<sensor_msgs::PointCloud2>("/cloud_debug", 100);
  ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseArray>("/poseArrayTopic", 10);
  ros::Publisher pub_trajectory = nh.advertise<visualization_msgs::Marker>("/trajectory_marker", 100);
  ros::Publisher pub_pose_number = nh.advertise<visualization_msgs::MarkerArray>("/pose_number", 100);

  string data_path;
  double downsample_size, marker_size;
  int pcd_name_fill_num, gap, start;

  nh.getParam("data_path", data_path);
  nh.getParam("downsample_size", downsample_size);
  nh.getParam("pcd_name_fill_num", pcd_name_fill_num);
  nh.getParam("marker_size", marker_size);
  nh.getParam("gap", gap);
  nh.getParam("start", start);

	std::vector<Submap> submaps = get_submap_info(data_path+"config.yaml");
 
  sensor_msgs::PointCloud2 debugMsg, cloudMsg, outMsg;
  vector<mypcl::pose> pose_vec;
  cout<<data_path + "pose.json"<<endl;
  for(auto& submap: submaps) {
		vector<mypcl::pose> pose_tmp = mypcl::read_pose(submap.data_path + "pose.json", submap.rotation, submap.translation);
		submap.length = pose_tmp.size();
		pose_vec.insert(pose_vec.end(), pose_tmp.begin(), pose_tmp.end());
	}

  size_t pose_size = pose_vec.size();
  cout<<"pose size "<<pose_size<<endl;

  pcl::PointCloud<PointType2>::Ptr pc_surf(new pcl::PointCloud<PointType2>);
  pcl::PointCloud<PointType2>::Ptr color_full(new pcl::PointCloud<PointType2>);

  ros::Time cur_t;
  geometry_msgs::PoseArray parray;
  parray.header.frame_id = "camera_init";
  parray.header.stamp = cur_t;
  visualization_msgs::MarkerArray markerArray;
  pcl::VoxelGrid<PointType2> downsample;
  downsample.setLeafSize(downsample_size, downsample_size, downsample_size);
  // cout<<"push enter to view"<<endl;
  // getchar();
  for(size_t k = start/gap; k*gap < pose_size; k++)
  {
    
    size_t i = k * gap;
    displayProgressBar(i, pose_size);

    std::pair<int, int> index = get_submap_index(submaps, i);
    std::stringstream ss;
    ss << std::setw(pcd_name_fill_num) << std::setfill('0') << index.second;
    
    pcl::io::loadPCDFile(submaps[index.first].data_path + "pcd/" + ss.str() + ".pcd", *pc_surf);
    pcl::PointCloud<PointType2>::Ptr pc_filtered(new pcl::PointCloud<PointType2>);
    pcl::PointCloud<PointType2>::Ptr pc_filtered_tmp(new pcl::PointCloud<PointType2>);
    pc_filtered->resize(pc_surf->points.size());

    int cnt = 0;
    for(size_t j = 0; j < pc_surf->points.size(); j++)
    {
      pc_filtered->points[cnt] = pc_surf->points[j];
      cnt++;
    }
    pc_filtered->resize(cnt);
    
    transform_pointcloud_rgb(*pc_filtered, *pc_filtered, pose_vec[i].t, pose_vec[i].q);


    downsample.setInputCloud(pc_filtered);
    downsample.filter(*pc_filtered_tmp);
    *pc_filtered = *pc_filtered_tmp;
    *color_full += *pc_filtered;
    if(1)
      if(i % int(pose_size/10) == 0){
        downsample.setInputCloud(color_full);
        downsample.filter(*pc_filtered_tmp);
        *color_full = *pc_filtered_tmp;

        pcl::io::savePCDFileASCII(data_path+"out.pcd", *color_full);
      }
    // downsample_voxel(*pc_filtered, downsample_size);

    pcl::toROSMsg(*pc_filtered, cloudMsg);
    cloudMsg.header.frame_id = "camera_init";
    cloudMsg.header.stamp = cur_t;
    pub_map.publish(cloudMsg);

    geometry_msgs::Pose apose;
    apose.orientation.w = pose_vec[i].q.w();
    apose.orientation.x = pose_vec[i].q.x();
    apose.orientation.y = pose_vec[i].q.y();
    apose.orientation.z = pose_vec[i].q.z();
    apose.position.x = pose_vec[i].t(0);
    apose.position.y = pose_vec[i].t(1);
    apose.position.z = pose_vec[i].t(2);
    parray.poses.push_back(apose);
    pub_pose.publish(parray);

    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // transform.setOrigin(tf::Vector3(pose_vec[i].t(0), pose_vec[i].t(1), pose_vec[i].t(2)));
    // tf::Quaternion q(pose_vec[i].q.x(), pose_vec[i].q.y(), pose_vec[i].q.z(), pose_vec[i].q.w());
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_init", "turtle_name"));

    // publish pose trajectory
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_init";
    marker.header.stamp = cur_t;
    marker.ns = "basic_shapes";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = pose_vec[i].t(0);
    marker.pose.position.y = pose_vec[i].t(1);
    marker.pose.position.z = pose_vec[i].t(2);
    pose_vec[i].q.normalize();
    marker.pose.orientation.x = pose_vec[i].q.x();
    marker.pose.orientation.y = pose_vec[i].q.y();
    marker.pose.orientation.z = pose_vec[i].q.x();
    marker.pose.orientation.w = pose_vec[i].q.w();
    marker.scale.x = marker_size; // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.y = marker_size;
    marker.scale.z = marker_size;
    marker.color.r = float(1-float(i)/pose_size);
    marker.color.g = float(float(i)/pose_size);
    marker.color.b = float(float(i)/pose_size);
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    pub_trajectory.publish(marker);

    // publish pose number
    visualization_msgs::Marker marker_txt;
    marker_txt.header.frame_id = "camera_init";
    marker_txt.header.stamp = cur_t;
    marker_txt.ns = "marker_txt";
    marker_txt.id = i; // Any marker sent with the same namespace and id will overwrite the old one
    marker_txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    ostringstream str;
    str << i;
    marker_txt.text = str.str();
    marker.action = visualization_msgs::Marker::ADD;
    marker_txt.action = visualization_msgs::Marker::ADD;
    marker_txt.pose.position.x = pose_vec[i].t(0)+marker_size;
    marker_txt.pose.position.y = pose_vec[i].t(1)+marker_size;
    marker_txt.pose.position.z = pose_vec[i].t(2);
    marker_txt.pose.orientation.x = pose_vec[i].q.x();
    marker_txt.pose.orientation.y = pose_vec[i].q.y();
    marker_txt.pose.orientation.z = pose_vec[i].q.x();
    marker_txt.pose.orientation.w = 1.0;
    marker_txt.scale.x = marker_size;
    marker_txt.scale.y = marker_size;
    marker_txt.scale.z = marker_size;
    marker_txt.color.r = 1.0f;
    marker_txt.color.g = 1.0f;
    marker_txt.color.b = 1.0f;
    marker_txt.color.a = 1.0;
    marker_txt.lifetime = ros::Duration();

    if(i%GAP == 0) markerArray.markers.push_back(marker_txt);
    pub_pose_number.publish(markerArray);

    ros::Duration(0.0001).sleep();

  }

  ros::Rate loop_rate(1);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}