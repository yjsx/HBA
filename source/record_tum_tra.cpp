#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <fstream>  
using namespace std;
ofstream outfile;


void chatterCallback(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
  outfile<<laserOdometry->header.stamp
        <<" "<<laserOdometry->pose.pose.position.x
        <<" "<<laserOdometry->pose.pose.position.y
        <<" "<<laserOdometry->pose.pose.position.z  
        <<" "<<laserOdometry->pose.pose.orientation.x
        <<" "<<laserOdometry->pose.pose.orientation.y
        <<" "<<laserOdometry->pose.pose.orientation.z
        <<" "<<laserOdometry->pose.pose.orientation.w<<endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "record_tra", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  outfile.open(argv[2], ios::out | ios::trunc );
  ROS_INFO("start recording");
  ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>(argv[1], 1000, chatterCallback);
  ros::spin();
  return 0;
}
