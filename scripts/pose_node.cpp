#include <ros/ros.h>
#include "TiagoBears_PoseEstimation/pose_estimator.h"


// bool cb (TiagoBears_PoseEstimation::PoseEstimation::Request &req, TiagoBears_PoseEstimation::PoseEstimation::Response &res){
//     ROS_INFO("I heard: [service callback]");
//     return true;
// }


bool service_callback(TiagoBears_PoseEstimation::PoseEstimation::Request &req, TiagoBears_PoseEstimation::PoseEstimation::Response &res)
{
  sensor_msgs::PointCloud2 msg_cloud;
  //pcl::PCLPointCloud2ConstPtr msg_cloud;
  sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/xtion/depth_registered/points", ros::Duration(10));
    if (msg == NULL)
        ROS_INFO("No point clound messages received");
    else
        msg_cloud = *msg;

  pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("cube.pcd", cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *model_cloud); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>
  

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_filtered_cloud(new pcl::PointCloud< pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr msg_cloud_pcl(new pcl::PointCloud< pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filterd_vox(new pcl::PointCloud< pcl::PointXYZ>);

  // Convert from PointCloud2 to PointCloud
  //pcl::fromPCLPointCloud2 (msg_cloud, *msg_cloud_pcl);
  pcl::fromROSMsg(msg_cloud, *msg_cloud_pcl);

  pcl::VoxelGrid<pcl::PointXYZ> vox;
  vox.setInputCloud (msg_cloud_pcl);
  vox.setLeafSize (0.003f, 0.003f, 0.003f);
  vox.filter (*cloud_filterd_vox);


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(new pcl::PointCloud< pcl::PointXYZ>);

  // Set up CropBox and filter input PointCloud
  pcl::CropBox<pcl::PointXYZ> crop;
  crop.setInputCloud (cloud_filterd_vox);
  // y and z parameter seem to crop same dimension just inverted?
  crop.setMin(Eigen::Vector4f(-0.6, -0.6, -1, 0.));
  crop.setMax(Eigen::Vector4f(+0.6, +1.1, +1, 0.));
  crop.filter (*cloud_cropped);
  ROS_INFO("[filtered]");


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.005f);
  // Segment largest planar component from cropped cloud
  seg.setInputCloud (cloud_cropped);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0)
  {
    ROS_INFO ("Could not estimate a planar model for the given dataset.") ;
  }


  // Extract planar inliers from input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud_cropped);
  extract.setIndices(inliers);
  extract.setNegative (false);

  // Get points associated with planar surface
  extract.filter (*cloud_plane);

  // Remove the planar inliers and extract rest
  extract.setNegative (true);
  extract.filter (*cloud_seg);
  

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cubes (new pcl::PointCloud<pcl::PointXYZ>(*cloud_seg));

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  tree->setInputCloud (cloud_cubes);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
  ece.setClusterTolerance (0.004f); //cluster_tolerance 
  ece.setMinClusterSize (150); //cluster_min_size
  ece.setMaxClusterSize (1500); //cluster_max_size
  ece.setSearchMethod (tree);
  ece.setInputCloud (cloud_cubes);
  ece.extract (cluster_indices);
  ROS_INFO_STREAM("Size of cluster_indices: " << cluster_indices.size());
  

  std::vector<sensor_msgs::PointCloud2::Ptr> pc2_clusters;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_vec;

  pcl::ExtractIndices<pcl::PointXYZ> extract_cluster;
  
  for (auto& cluster_idx : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr idx (new pcl::PointIndices);
    *idx = cluster_idx;
    extract_cluster.setInputCloud(cloud_cubes);
    extract_cluster.setIndices(idx);
    extract_cluster.setNegative (false);
    extract_cluster.filter (*cloud_cluster);
    clusters_vec.push_back(cloud_cluster);
  }
  std::vector <nav_msgs::Odometry> pose_vec;
    
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  
  //ICP
  std::vector<double> score_vec(28, 0.0);
  //std::vector<double> score_vec;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > icp_vec;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations (1000);
  icp.setTransformationEpsilon (1e-9);
  //icp.setMaxCorrespondenceDistance (0.003);
  //icp.setRANSACOutlierRejectionThreshold (1.);
  
  std::vector<nav_msgs::Odometry> estimated_pose_vec(28);

  int i=0;
  for (auto& clouds : clusters_vec)
  {
    icp.setInputSource (clouds);
    icp.setInputTarget (model_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud< pcl::PointXYZ>);

    icp.align (*cloud_icp);
    icp_vec.push_back(cloud_icp);
    double score = icp.getFitnessScore();
    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    Eigen::Matrix4f icp_transformation = icp.getFinalTransformation ();
    score_vec.push_back(score);
    //std::cout << " Prob: " << prob_vec.at(i) << std::endl;
    

    try{
      transformStamped = tfBuffer.lookupTransform("base_footprint","xtion_depth_frame",ros::Time(0),ros::Duration(0.1));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
    }

    Eigen::Matrix4d temp_matrix;
    temp_matrix = icp_transformation.cast<double>();
    Eigen::Affine3d pose_icp;
    pose_icp.matrix() = temp_matrix;
    //std::cout << " Matrix1: " << pose_icp.matrix() << std::endl;
    geometry_msgs::TransformStamped estimated_pose = tf2::eigenToTransform(pose_icp);
    //std::cout << " Matrix2: " << estimated_pose << std::endl;

    geometry_msgs::TransformStamped  pose_transformed;
    tf2::doTransform(estimated_pose, pose_transformed, transformStamped);
    
    Eigen::Affine3d pose_icp_transformed;
    tf2::doTransform(pose_icp, pose_icp_transformed, transformStamped);
    std::cout << " Matrix: " << pose_icp_transformed.matrix() << std::endl;
    
    nav_msgs::Odometry pose_odometry;

    pose_odometry.header = pose_transformed.header;
    pose_odometry.child_frame_id = pose_transformed.child_frame_id;
    pose_odometry.pose.pose.position.x = pose_transformed.transform.translation.x;
    pose_odometry.pose.pose.position.y = pose_transformed.transform.translation.y;
    pose_odometry.pose.pose.position.z = pose_transformed.transform.translation.z;
    pose_odometry.pose.pose.orientation = pose_transformed.transform.rotation;

    std::cout << " x: " << pose_odometry.pose.pose.position.x << std::endl;
    std::cout << " y: " << pose_odometry.pose.pose.position.y << std::endl;
    std::cout << " z: " << pose_odometry.pose.pose.position.z << std::endl;
    // tf::Transform tf_ref = tf::Transform(tf::Matrix3x3(icp_transformation(0,0),icp_transformation(0,1),icp_transformation(0,2),
    //                                                             icp_transformation(1,0),icp_transformation(1,1),icp_transformation(1,2),
    //                                                             icp_transformation(2,0),icp_transformation(2,1),icp_transformation(2,2)),
    //                                               tf::Vector3(icp_transformation(0,3),icp_transformation(1,3),icp_transformation(2,3)));
    // //tf::Vector3 trans = tf_ref.getOrigin();
    // //tf::Quaternion rot = tf_ref.getRotation();
    //target_pose = cam_to_base * tf_ref;
    //geometry_msgs::Pose pos_tmp;
    //tf::poseTFToMsg(target_pose, pos_tmp);


    estimated_pose_vec.at(i) = pose_odometry;
    
    res.poseArray[i] = pose_odometry;

    pose_vec.push_back(pose_odometry);
    //geometry_msgs::Pose pose_msg;
    //tf2::doTransform()
    i++;
  }
  return true;
  //res = estimated_pose_vec;
}


int main(int argc, char **argv)
{
    ROS_INFO("I heard: [node]");
    ros::init(argc, argv, "TiagoBears_PoseEstimation");
    ROS_INFO("I heard: [node started]");
    ros::NodeHandle nh;
    PoseEstimator pose_estimator(nh);
    ros::ServiceServer pose_server = nh.advertiseService("PoseEstimation", service_callback)  ;
    ros::spin();
}

