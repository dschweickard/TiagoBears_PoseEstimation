#ifndef PSE_ESTIMATION
#define POSE_ESTIMATION

// general headers
#include<iostream>
#include<string>
#include<sstream>
#include<vector>

// ROS headers
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

// PCL related headers
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// the class and functions prototypes
class PoseEstimator{
    // an array of publishers each of which publish the pose of a cube
    std::vector<ros::Publisher> pose_publishers;
    // a subscriber to get the point cloud
    ros::Subscriber point_cloud_subscriber;
public:
    // the constructor
    PoseEstimator(ros::NodeHandle n);
    void pcl_callback(const PointCloud::ConstPtr& msg);
};

#endif