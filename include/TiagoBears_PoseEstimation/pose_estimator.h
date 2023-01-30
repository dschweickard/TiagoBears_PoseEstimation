#ifndef PSE_ESTIMATION
#define POSE_ESTIMATION

// general headers
#include<iostream>
#include<string>
#include<sstream>
#include<vector>
#include <boost/filesystem.hpp>
// ROS headers
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

// PCL related headers
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/crop_box.h>

#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud2;
// the class and functions prototypes
class PoseEstimator{
    // an array of publishers each of which publish the pose of a cube
    std::vector<ros::Publisher> pose_publishers;
    // a subscriber to get the point cloudS
    ros::Subscriber point_cloud_subscriber;
    // point cloud cache
    PointCloud old_pcl;
    PointCloud cloud;
    PointCloud2 blob;
    // publisher for debugging
    ros::Publisher pub_model_cloud;
    ros::Publisher pub_cloud_debug;
public:
    // the constructor
    PoseEstimator(ros::NodeHandle n);
    void pcl_callback(const pcl::PCLPointCloud2ConstPtr& msg_cloud);
};

#endif