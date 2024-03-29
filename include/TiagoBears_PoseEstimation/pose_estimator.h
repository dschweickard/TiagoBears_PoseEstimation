#ifndef PSE_ESTIMATION
#define POSE_ESTIMATION

// general headers
#include<iostream>
#include<string>
#include<sstream>
#include<vector>
#include <boost/filesystem.hpp>
#include <thread>

// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <TiagoBears_PoseEstimation/PoseEstimation.h>
#include <TiagoBears_PoseEstimation/TableCornerDetection.h>


// PCL related headers
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
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
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/common/common.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
//#include <pcl/visualization/cloud_viewer.h>

#include <vtkPolyLine.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud2;
typedef int counter;

// the class and functions prototypes
class PoseEstimator{
    // an array of publishers each of which publish the pose of a cube
    std::vector<ros::Publisher> pose_publishers;
    std::vector<ros::Publisher> cloud_publishers;
    // a subscriber to get the point cloudS
    ros::Subscriber point_cloud_subscriber;
    // point cloud cache
    PointCloud old_pcl;
    PointCloud cloud;
    PointCloud2 blob;
    // publisher for debugging
    ros::Publisher pub_cloud_debug;
    ros::Publisher pub_icp_debug;
    ros::Publisher pub_pose_debug;
    ros::Publisher pub_pose_array;
    //Server
    //ros::ServiceServer pose_server;
public:
    // the constructor
    PoseEstimator(ros::NodeHandle n);
    
    void pcl_callback(const pcl::PCLPointCloud2ConstPtr& msg_cloud);

    bool service_callback_poses(TiagoBears_PoseEstimation::PoseEstimation::Request &req, 
                            TiagoBears_PoseEstimation::PoseEstimation::Response &res);
    
    bool service_callback_corners(TiagoBears_PoseEstimation::TableCornerDetection::Request &reqC,
                                    TiagoBears_PoseEstimation::TableCornerDetection::Response &resC);
};



#endif