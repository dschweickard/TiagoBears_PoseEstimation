#include <ros/ros.h>
#include <TiagoBears_PoseEstimation/pose_estimator.h>
#include <TiagoBears_PoseEstimation/PoseEstimation.h>



tf2::Matrix3x3 CorrectRotMatrix(tf2::Matrix3x3 &matrix){
  // conver them to Eigen matrix
  Eigen::MatrixXf eigenMatrix = Eigen::MatrixXf::Zero(3,3);
  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
      eigenMatrix(i,j) = matrix[i][j];
    }
  }
  for (int i=0;i<3; i++){
    // take a sub matrix with size (3-i)x(3-i)
    Eigen::MatrixXf subMatrix = eigenMatrix.block(0,0,3-i,3-i);
    // update the submatrix to have absolute values
    subMatrix = subMatrix.cwiseAbs();
    // find the max element in the sub matrix
    int max_x, max_y;
    float maxVal = subMatrix.maxCoeff(&max_x, &max_y);
    // swap the rows and columns such that the max value is in (3-i), (3-i) position
    eigenMatrix.row(2-i).swap(eigenMatrix.row(max_x));
    eigenMatrix.col(2-i).swap(eigenMatrix.col(max_y));
    // multipy the row with -1 if the max value is negative
    if (eigenMatrix(2-i,2-i) < 0){
      eigenMatrix.row(2-i) = -1 * eigenMatrix.row(2-i);
    }
  }
  // convert back to tf2::Matrix3x3
  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
      matrix[i][j] = eigenMatrix(i,j);
    }
  }
  return matrix;
}


std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> LCCP_segmentation (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud){

float voxel_resolution = 0.0025f; //0.0025f
float seed_resolution = 0.005f; //0.005f
float color_importance = 0.5f;
float spatial_importance = 0.1f;
float normal_importance = 1.0f;


// LCCP Segmentation
float concavity_tolerance_threshold = 10;
float smoothness_threshold = 0.1; //0.1
std::uint32_t min_segment_size = 5;
bool use_extended_convexity = false;
bool use_sanity_criterion = false;
int k_factor = 0;//1

int th_points = 150;


// // Split pointcloud to fit SuperVoxelClustering input format
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
// pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
pcl::copyPointCloud(*input_cloud, *cloud_RGB);
// pcl::copyPointCloud(*input_cloud, *cloud_normals);

// // Compute SuperVoxel Clustering
pcl::SupervoxelClustering<pcl::PointXYZRGB> super(voxel_resolution, seed_resolution);
super.setUseSingleCameraTransform(false);
super.setInputCloud(cloud_RGB);
// super.setNormalCloud(cloud_normals);

super.setColorImportance(color_importance);
super.setSpatialImportance(spatial_importance);
super.setNormalImportance(normal_importance);

std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> supervoxel_clusters;
PCL_INFO("Extracting supervoxels\n");
super.extract(supervoxel_clusters);
super.refineSupervoxels(2, supervoxel_clusters);

PCL_INFO("Getting supervoxel adjacency\n");
std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
super.getSupervoxelAdjacency(supervoxel_adjacency);

pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();


/// Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visulization)
pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<pcl::PointXYZRGB>::makeSupervoxelNormalCloud (supervoxel_clusters);

PCL_INFO("Starting LCCP Segmentation\n");
pcl::LCCPSegmentation<pcl::PointXYZRGB> lccp;
lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
lccp.setSanityCheck(use_sanity_criterion);
lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
lccp.setKFactor(k_factor);
lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
lccp.setMinSegmentSize(min_segment_size);
lccp.segment();

PCL_INFO("Interpolation voxel cloud -> input cloud and relabeling\n");
pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
lccp.relabelCloud(*lccp_labeled_cloud);
pcl::LCCPSegmentation<pcl::PointXYZRGB>::SupervoxelAdjacencyList sv_adjacency_list;
lccp.getSVAdjacencyList (sv_adjacency_list);



std::vector<pcl::PointCloud<pcl::PointXYZRGB>> object_vec;
//object_vec.resize(0);

for (int i = 0; i < lccp_labeled_cloud->points.size(); ++i)
{
  
  uint32_t idx = lccp_labeled_cloud->points.at(i).label;

  if(idx >= object_vec.size()) 
    object_vec.resize(idx+1);
    //PCL_INFO("Resize 1\n");

  pcl::PointXYZRGB temp_point;
  //PCL_INFO("try access\n");
  temp_point = cloud_RGB->points.at(i);
  //PCL_INFO("Saved temp point\n");
  object_vec.at(idx).points.push_back(temp_point);

  //PCL_INFO("pushed point\n");
} 

 std::cout << "LCCP Cluster size:" << object_vec.size() << std::endl;


//pcl::toPCLPointCloud2 (*cloud_cluster1, output_label_cloud2);
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> output_vec;

for (int j =0; j < object_vec.size(); j++)
{

  object_vec.at(j).header.frame_id = input_cloud->header.frame_id;
  if (object_vec.at(j).size() > th_points){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object =  object_vec.at(j).makeShared();
    output_vec.push_back(object);
  }

}

return output_vec;
}



bool service_callback(TiagoBears_PoseEstimation::PoseEstimation::Request &req, TiagoBears_PoseEstimation::PoseEstimation::Response &res)
{
  float leaf_size = 0.0025f;

  float crop_x_min = -0.6f;
  float crop_x_max = +0.6f;
  float crop_y_min = -0.6f;
  float crop_y_max = +1.1f;
  float crop_z_min = -1.0f;
  float crop_z_max = +1.0f;
  float plane_seg_dist_treshold = 0.0065f;
  
  int MeanK = 50;
  float StddevMulThresh = 1.0f;

  float cluster_tolerance = 0.0035f;
  int cluster_size_min = 150;
  int cluster_size_max = 1100;



  sensor_msgs::PointCloud2 msg_cloud;
  //pcl::PCLPointCloud2ConstPtr msg_cloud;
  sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/xtion/depth_registered/points", ros::Duration(10));
  if (msg == NULL)
      ROS_INFO("No point clound messages received");
  else
        msg_cloud = *msg;
  ROS_INFO_STREAM("FrameID  " << msg->header.frame_id);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("src/TiagoBears_PoseEstimation/models/cube.pcd", cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *model_cloud); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>
  

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_filtered_cloud(new pcl::PointCloud< pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr msg_cloud_pcl(new pcl::PointCloud< pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filterd_vox(new pcl::PointCloud< pcl::PointXYZRGB>);

  // Convert from PointCloud2 to PointCloud
  //pcl::fromPCLPointCloud2 (msg_cloud, *msg_cloud_pcl);
  pcl::fromROSMsg(msg_cloud, *msg_cloud_pcl);

  pcl::VoxelGrid<pcl::PointXYZRGB> vox;
  vox.setInputCloud (msg_cloud_pcl);
  vox.setLeafSize (leaf_size,leaf_size,leaf_size);
  vox.filter (*cloud_filterd_vox);


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cropped(new pcl::PointCloud< pcl::PointXYZRGB>);

  // Set up CropBox and filter input PointCloud
  pcl::CropBox<pcl::PointXYZRGB> crop;
  crop.setInputCloud (cloud_filterd_vox);
  // y and z parameter seem to crop same dimension just inverted?
  crop.setMin(Eigen::Vector4f(crop_x_min, crop_y_min, crop_z_min, 0.));
  crop.setMax(Eigen::Vector4f(crop_x_max, crop_y_max, crop_z_max, 0.));
  crop.filter (*cloud_cropped);
  ROS_INFO("[filtered]");


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_seg (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (plane_seg_dist_treshold);
  // Segment largest planar component from cropped cloud
  seg.setInputCloud (cloud_cropped);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0)
  {
    ROS_INFO ("Could not estimate a planar model for the given dataset.") ;
  }


  // Extract planar inliers from input cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud (cloud_cropped);
  extract.setIndices(inliers);
  extract.setNegative (false);

  // Get points associated with planar surface
  extract.filter (*cloud_plane);

  // Remove the planar inliers and extract rest
  extract.setNegative (true);
  extract.filter (*cloud_seg);
  


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cubes (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud_seg);
  sor.setMeanK (MeanK);
  sor.setStddevMulThresh (StddevMulThresh);
  sor.filter (*cloud_cubes);
  

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_vec;
  clusters_vec = LCCP_segmentation(cloud_cubes);
  // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  // tree->setInputCloud (cloud_cubes);

  // std::vector<pcl::PointIndices> cluster_indices;
  // pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
  // ece.setClusterTolerance (0.003f); //cluster_tolerance 
  // ece.setMinClusterSize (200); //cluster_min_size
  // ece.setMaxClusterSize (1100); //cluster_max_size
  // ece.setSearchMethod (tree);
  // ece.setInputCloud (cloud_cubes);
  // ece.extract (cluster_indices);
  // //ROS_INFO_STREAM("Size of cluster_indices: " << cluster_indices.size());
    

  // pcl::ExtractIndices<pcl::PointXYZRGB> extract_cluster;
  // for (auto& cluster_idx : cluster_indices)
  // {
  //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
  //     pcl::PointIndices::Ptr idx (new pcl::PointIndices);
  //     *idx = cluster_idx;
  //     extract_cluster.setInputCloud(cloud_cubes);
  //     extract_cluster.setIndices(idx);
  //     extract_cluster.setNegative (false);
  //     extract_cluster.filter (*cloud_cluster);
  //     clusters_vec.push_back(cloud_cluster);
  // }

  std::vector <nav_msgs::Odometry> pose_vec;
    
  tf2_ros::Buffer cloudBuffer;
  geometry_msgs::TransformStamped transformStampedICPPose;
  tf2_ros::TransformListener tfListenerCloud(cloudBuffer);
  
  //ICP
  std::vector<double> score_vec(28, 0.0);
  //std::vector<double> score_vec;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > icp_vec;

  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setMaximumIterations (10000);
  icp.setTransformationEpsilon (1e-9);
  //icp.setMaxCorrespondenceDistance (0.003);
  //icp.setRANSACOutlierRejectionThreshold (1.);
  
  std::vector<nav_msgs::Odometry> estimated_pose_vec(28);

  int i=0;
  for (auto& clouds : clusters_vec)
  {
    icp.setInputSource (model_cloud);
    icp.setInputTarget (clouds);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_icp(new pcl::PointCloud< pcl::PointXYZRGB>);

    icp.align (*cloud_icp);
    icp_vec.push_back(cloud_icp);
    double score = icp.getFitnessScore();
    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    Eigen::Matrix4f icp_transformation = icp.getFinalTransformation ();
    score_vec.push_back(score);
    //std::cout << " Prob: " << prob_vec.at(i) << std::endl;
    
    tf2::Vector3 position_cube = tf2::Vector3(icp_transformation(0,3),icp_transformation(1,3),icp_transformation(2,3));
      
    tf2::Matrix3x3 cube_rotation = tf2::Matrix3x3(icp_transformation(0,0),icp_transformation(0,1),icp_transformation(0,2),
                                                                   icp_transformation(1,0),icp_transformation(1,1),icp_transformation(1,2),
                                                                   icp_transformation(2,0),icp_transformation(2,1),icp_transformation(2,2));
      
    tf2::Quaternion cube_orientation;
    cube_rotation.getRotation(cube_orientation); 

    geometry_msgs::TransformStamped estimated_pose;

    estimated_pose.header.frame_id = msg->header.frame_id;
    estimated_pose.header.seq = msg->header.seq;
    estimated_pose.transform.translation = tf2::toMsg(position_cube);
    estimated_pose.transform.rotation = tf2::toMsg(cube_orientation);
    geometry_msgs::TransformStamped  pose_transformed;
    cloudBuffer.transform(estimated_pose, pose_transformed,"base_footprint",ros::Duration(1.));

    // get the rotation matrix from the quaternion of pose_transformed
    tf2::Quaternion q(pose_transformed.transform.rotation.x, pose_transformed.transform.rotation.y, pose_transformed.transform.rotation.z, pose_transformed.transform.rotation.w);
    // quaternion to rotation matrix
    tf2::Matrix3x3 m(q);
    // correct the rotation matrix
    tf2::Matrix3x3 m2 = CorrectRotMatrix(m);
    m2.getRotation(cube_orientation);

    nav_msgs::Odometry pose_odometry;
    pose_odometry.header = pose_transformed.header;
    //pose_odometry.child_frame_id = pose_transformed.child_frame_id;
    pose_odometry.pose.pose.position.x = pose_transformed.transform.translation.x;
    pose_odometry.pose.pose.position.y = pose_transformed.transform.translation.y;
    pose_odometry.pose.pose.position.z = pose_transformed.transform.translation.z;
    pose_odometry.pose.pose.orientation = pose_transformed.transform.rotation;

    std::cout << " x: " << pose_odometry.pose.pose.position.x << std::endl;
    std::cout << " y: " << pose_odometry.pose.pose.position.y << std::endl;
    std::cout << " z: " << pose_odometry.pose.pose.position.z << std::endl;


    estimated_pose_vec.at(i) = pose_odometry;
    
    res.poseArray[i] = pose_odometry;

    //pose_vec.push_back(pose_odometry);

    i++;
  }

  return true;

}


int main(int argc, char **argv)
{
    ROS_INFO("I heard: [node]");
    ros::init(argc, argv, "TiagoBears_PoseEstimation");
    ROS_INFO("I heard: [node started]");
    ros::NodeHandle nh;
    PoseEstimator pose_estimator(nh);
    // define a ros service server
    ros::ServiceServer service = nh.advertiseService("/TiagoBears/PoseEstimation", service_callback);
    ros::spin();
}

