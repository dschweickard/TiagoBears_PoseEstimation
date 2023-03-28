#include<TiagoBears_PoseEstimation/pose_estimator.h>
#include<TiagoBears_PoseEstimation/PoseEstimation.h>

PoseEstimator::PoseEstimator(ros::NodeHandle n){
    ROS_INFO("I heard: [constructor]");
    // initialize the publishers
    //for(int i=0; i<28; i++){
    
    //      char* topic;
    //      sprintf(topic, "/cube_%d_odom", i);
    //      pose_publishers.push_back(n.advertise<nav_msgs::Odometry>(topic,1));
    //}
    //ROS_INFO("I heard: [Publisher clouds]");
    //for(int j=0; j<28; j++){
    //       char* topic;
    //       sprintf(topic, "/cloud_cluster_%d", j);
     //      cloud_publishers.push_back(n.advertise<sensor_msgs::PointCloud2>(topic,1));
   // }
    //pcl::io::loadPCDFile ("/meshs/cubeModel.pcd", blob);
    //ROS_INFO("I heard: [readPCD]");
    //pcl::fromPCLPointCloud2 (blob, cloud);
    //ROS_INFO("I heard: [Convert]");

    // initialize the point cloud subscriber
    //ROS_INFO("I heard: [Subscriber]");
    point_cloud_subscriber=n.subscribe("/xtion/depth_registered/points", 1, &PoseEstimator::pcl_callback, this);
    //point_cloud_subscriber=n.subscribe("/bag/points", 1, &PoseEstimator::pcl_callback, this);
    
    //ROS_INFO("I heard: [Publisher1]");
    pub_cloud_debug = n.advertise<sensor_msgs::PointCloud2>("CloudFiltered", 1);
    //ROS_INFO("I heard: [Publisher2]");
    
    pub_pose_debug = n.advertise<nav_msgs::Odometry>("PlanePose", 1);
    //ROS_INFO("I heard: [Publisher3]");
    //pub_icp_debug = n.advertise<sensor_msgs::PointCloud2>("CloudICP", 1);
    //pose_server = n.advertiseService("PoseEstimation", &PoseEstimator::service_callback,this);

    //ROS_INFO("I heard: [Publisher 2]");
    //pub_cloud_cluster = n.advertise<sensor_msgs::PointCloud2>("cloudClusters", 1);

}



std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> EuclideanClustering (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud)
{

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

    tree->setInputCloud (input_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
    ece.setClusterTolerance (0.003f); //cluster_tolerance 
    ece.setMinClusterSize (200); //cluster_min_size
    ece.setMaxClusterSize (1100); //cluster_max_size
    ece.setSearchMethod (tree);
    ece.setInputCloud (input_cloud);
    ece.extract (cluster_indices);
    ROS_INFO_STREAM("Size of cluster_indices: " << cluster_indices.size());
    

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;

    pcl::ExtractIndices<pcl::PointXYZRGB> extract_cluster;
    for (auto& cluster_idx : cluster_indices)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointIndices::Ptr idx (new pcl::PointIndices);
      *idx = cluster_idx;
      extract_cluster.setInputCloud(input_cloud);
      extract_cluster.setIndices(idx);
      extract_cluster.setNegative (false);
      extract_cluster.filter (*cloud_cluster);
      clusters.push_back(cloud_cluster);
    }

  return clusters;
}


std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> LCCP (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud){

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

//PCL_INFO("Resize 0\n");

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






tf2::Matrix3x3 CorrectRotationMatrix(tf2::Matrix3x3 &matrix){
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



std::vector<nav_msgs::Odometry> estimatePose( std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& clustered_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& model_object)
{

std::vector<nav_msgs::Odometry> estimated_pose_vec(28);

float leaf = 0.01f;

pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

pcl::NormalEstimationOMP<pcl::PointXYZRGB,pcl::Normal> nest;
nest.setRadiusSearch (0.005);

pcl::PointCloud<pcl::Normal>::Ptr model_object_normals (new pcl::PointCloud<pcl::Normal>);
tree->setInputCloud(model_object);
nest.setSearchMethod(tree);
nest.setInputCloud (model_object);
nest.compute (*model_object_normals);


pcl::FPFHEstimationOMP<pcl::PointXYZRGB,pcl::Normal, pcl::FPFHSignature33> fest;
fest.setRadiusSearch (0.025);

pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_object_features (new pcl::PointCloud<pcl::FPFHSignature33>);
fest.setInputCloud (model_object);
fest.setInputNormals (model_object_normals);
fest.compute (*model_object_features);


pcl::SampleConsensusPrerejective<pcl::PointXYZRGB,pcl::PointXYZRGB,pcl::FPFHSignature33> align;
align.setMaximumIterations (50000); // Number of RANSAC iterations
align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
align.setCorrespondenceRandomness (5); // Number of nearest features to use
align.setSimilarityThreshold (0.95f); // Polygonal edge length similarity threshold
align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis


pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
icp.setMaximumIterations (1000000);
icp.setTransformationEpsilon (1e-12);

tf2_ros::Buffer cloudBuffer;
tf2_ros::TransformListener tfListenerCloud(cloudBuffer);

int j = 0;

for (auto& cluster : clustered_cloud){

  pcl::PointCloud<pcl::Normal>::Ptr cluster_normals (new pcl::PointCloud<pcl::Normal>);
  tree->setInputCloud(cluster);
  nest.setSearchMethod(tree);
  nest.setInputCloud (cluster);
  nest.compute (*cluster_normals);

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr cluster_features (new pcl::PointCloud<pcl::FPFHSignature33>);
  fest.setInputCloud (cluster);
  fest.setInputNormals (cluster_normals);
  fest.compute (*cluster_features);

  // Perform alignment

  align.setInputSource (model_object);
  align.setSourceFeatures (model_object_features);
  align.setInputTarget (cluster);
  align.setTargetFeatures (cluster_features);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_object_aligned (new pcl::PointCloud<pcl::PointXYZRGB>);

  align.align (*model_object_aligned);

  Eigen::Matrix4f transformation_guess = align.getFinalTransformation ();

  icp.setInputSource (model_object);
  icp.setInputTarget (cluster);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_icp(new pcl::PointCloud< pcl::PointXYZRGB>);

  icp.align (*model_object_aligned,transformation_guess);

  double score = icp.getFitnessScore();
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  //Eigen::Matrix4f icp_transformation = icp.getFinalTransformation ();

  Eigen::Matrix4f icp_transformation = transformation_guess;
  tf2::Vector3 position_cube = tf2::Vector3(icp_transformation(0,3),icp_transformation(1,3),icp_transformation(2,3));

  tf2::Matrix3x3 cube_rotation = tf2::Matrix3x3(icp_transformation(0,0),icp_transformation(0,1),icp_transformation(0,2),
                                                                icp_transformation(1,0),icp_transformation(1,1),icp_transformation(1,2),
                                                                icp_transformation(2,0),icp_transformation(2,1),icp_transformation(2,2));

  tf2::Quaternion cube_orientation;
  cube_rotation.getRotation(cube_orientation); 

  geometry_msgs::TransformStamped estimated_pose;


  estimated_pose.header.frame_id = cluster->header.frame_id;
  estimated_pose.header.seq = cluster->header.seq;
  estimated_pose.transform.translation = tf2::toMsg(position_cube);
  estimated_pose.transform.rotation = tf2::toMsg(cube_orientation);

  geometry_msgs::TransformStamped  pose_transformed;
  cloudBuffer.transform(estimated_pose, pose_transformed,"base_footprint",ros::Duration(1.));

  // get the rotation matrix from the quaternion of pose_transformed
  tf2::Quaternion q(pose_transformed.transform.rotation.x, pose_transformed.transform.rotation.y, pose_transformed.transform.rotation.z, pose_transformed.transform.rotation.w);
  // quaternion to rotation matrix
  tf2::Matrix3x3 m(q);
  // correct the rotation matrix
  tf2::Matrix3x3 m2 = CorrectRotationMatrix(m);
  m2.getRotation(cube_orientation);

  nav_msgs::Odometry pose_odometry;
  pose_odometry.header = pose_transformed.header;
  pose_odometry.header.frame_id = "base_footprint";
  //pose_odometry.child_frame_id = pose_transformed.child_frame_id;
  pose_odometry.pose.pose.position.x = pose_transformed.transform.translation.x;
  pose_odometry.pose.pose.position.y = pose_transformed.transform.translation.y;
  pose_odometry.pose.pose.position.z = pose_transformed.transform.translation.z;
  pose_odometry.pose.pose.orientation = tf2::toMsg(cube_orientation);


  //std::cout << " x: " << pose_odometry.pose.pose.position.x << std::endl;
  //std::cout << " y: " << pose_odometry.pose.pose.position.y << std::endl;
  //std::cout << " z: " << pose_odometry.pose.pose.position.z << std::endl;


  estimated_pose_vec.at(j) = pose_odometry;

  j++;
} 



return estimated_pose_vec;

}


void PoseEstimator::pcl_callback(const pcl::PCLPointCloud2ConstPtr& msg_cloud){
    

    //boost::filesystem::path full_path(boost::filesystem::current_path());
    //std::cout << "Current path is : " << full_path << std::endl;

    //ROS_INFO("I heard: [Load Model]");
    ROS_INFO_STREAM("FrameID  " << msg_cloud->header.frame_id);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile ("src/TiagoBears_PoseEstimation/models/cube.pcd", cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, *model_cloud); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>
    

    //pub_model_cloud.publish(cloud_blob);
    //pcl::PointCloud<pcl::PointXYZRGB> scene_cloud;
    //pcl::io::loadPCDFile ("src/TiagoBears_PoseEstimation/robot_scene.pcd", scene_cloud);
    //scene_cloud.header.frame_id = msg_cloud->header.frame_id;
    //pub_cloud_debug.publish(scene_cloud);
    // pub_icp_debug.publish(scene_cloud);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_filtered_cloud(new pcl::PointCloud< pcl::PointXYZRGB>);


    //Initialize new Pointers for cropped PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr msg_cloud_pcl(new pcl::PointCloud< pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filterd_vox(new pcl::PointCloud< pcl::PointXYZRGB>);

    // Convert from PointCloud2 to PointCloud
    pcl::fromPCLPointCloud2 (*msg_cloud, *msg_cloud_pcl);
    
    //pcl::copyPointCloud(scene_cloud, *msg_cloud_pcl);
     
    //pcl::copyPointCloud(scene_cloud, *cloud_filterd_vox); 

    ROS_INFO_STREAM(":Size Input PointCloud  " << msg_cloud_pcl->size());

    // pcl::io::savePCDFile ("robot_scene.pcd", *msg_cloud_pcl);

    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud (msg_cloud_pcl);
    vox.setLeafSize (0.002f, 0.002f, 0.002f);
    vox.filter (*cloud_filterd_vox);
    
    vox.setInputCloud(model_cloud);
    vox.filter(*model_cloud);

    ROS_INFO_STREAM(":Size voxel filtered PointCloud  " << cloud_filterd_vox->size());
    
    ROS_INFO_STREAM(":Size model PointCloud  " << model_cloud->size());

    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cropped(new pcl::PointCloud< pcl::PointXYZRGB>);

    // Set up CropBox and filter input PointCloud
    pcl::CropBox<pcl::PointXYZRGB> crop;
    crop.setInputCloud (cloud_filterd_vox);
    // y and z parameter seem to crop same dimension just inverted?
    crop.setMin(Eigen::Vector4f(-0.6, -0.6, -1., 0.));
    crop.setMax(Eigen::Vector4f(+0.6, +1.1, +1., 0.));
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
    seg.setDistanceThreshold (0.0065f);
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
    




    
    
    ROS_INFO_STREAM(":Size PointCLoud of cubes  " << cloud_seg->size());
    //SuperVoxelClustering(cloud_seg);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cubes (new pcl::PointCloud<pcl::PointXYZRGB>(*cloud_seg));


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cubes_filtered (new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_cubes);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_cubes);
    //pub_cloud_debug.publish(cloud_cubes);
    sor.setInputCloud(cloud_plane);
    sor.filter(*cloud_plane);


    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_vec;
    //clusters_vec = EuclideanClustering(cloud_cubes);

    //std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_vec;
    clusters_vec = LCCP(cloud_cubes);

    //pub_cloud_debug.publish(clusters_vec.at(1));
    //for(int i=0; i<clusters_vec.size();++i){
    //  cloud_publishers[i].publish(clusters_vec[i]);
    //} 
    
    
    //clusters_vec = RegionGrowingClustering (cloud_cubes);
    ROS_INFO_STREAM("Size of clusters: " << clusters_vec.size());
    
    //std::vector<nav_msgs::Odometry> estimated_pose_vec(28);

    //estimated_pose_vec = estimatePose(clusters_vec, model_cloud);

    //for(int i=0; i<estimated_pose_vec.size();++i){
    //  pose_publishers[i].publish(estimated_pose_vec[i]);
    //}

    //sensor_msgs::PointCloud2::Ptr cluster_out(new sensor_msgs::PointCloud2);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_out(new pcl::PointCloud<pcl::PointXYZ>);
    //*cluster_out = *clusters.at(3);
    //ROS_INFO_STREAM("Size of pointcloud: " << cluster_out->size());

    
    //ROS_INFO_STREAM("Size of pc2_clusters: " << cluster_out.);

    //listener.lookupTransform("base_footprint", msg_cloud->header.frame_id, ros::Time(0), transform);  
    
    //pcl_ros::transformPointCloud(*cloud_plane,*cloud_transformed,transform);
    //pub_cloud_debug.publish(cloud_transformed);
    //tf2_ros::Buffer tfBuffer;
    //tf2_ros::TransformListener tfListener(tfBuffer);
    //geometry_msgs::TransformStamped transStamped;

    // sensor_msgs::PointCloud2 cloud_transformed;
    // sensor_msgs::PointCloud2 msg_cloud_2;
    // pcl::toROSMsg(*msg_cloud_pcl,msg_cloud_2);


    //tf2_ros::TransformListener tfListenerCloud(cloudBuffer);
      //ROS_INFO("I heard: [Set up tf]");

    //const auto scan_tf = cloudBuffer.lookupTransform("base_footprint",msg_cloud->header.frame_id, ros::Time(0));
    
    //Eigen::Matrix4f trans_mat = tf2::transformToEigen(scan_tf.transform).matrix();
    
    //pcl_ros::transformPointCloud(trans_mat, msg_cloud_2,cloud_transformed);

    //pub_cloud_debug.publish(cloud_transformed);
    //cloudBuffer.transform(msg_cloud, cloud_transformed, "base_footprint",ros::Duration(1.));
        //ROS_INFO("I heard: [cloud transformed]");

    // pcl::PointXYZRGB minPt, maxPt;
    // pcl::getMinMax3D (*cloud_plane, minPt, maxPt);

    // std::cout << "Max x: " << maxPt.x << std::endl;
    // std::cout << "Max y: " << maxPt.y << std::endl;
    // std::cout << "Max z: " << maxPt.z << std::endl;
    // std::cout << "Min x: " << minPt.x << std::endl;
    // std::cout << "Min y: " << minPt.y << std::endl;
    // std::cout << "Min z: " << minPt.z << std::endl;
    
    // tf2::Vector3 position_corner_point = tf2::Vector3(minPt.x, minPt.y, minPt.z);
    // tf2::Quaternion orientation_corner_point = tf2::Quaternion(0.,0.,0.,0.);




    // geometry_msgs::TransformStamped estimated_corner_point;
    // estimated_corner_point.header.frame_id = cloud_plane->header.frame_id;
    // estimated_corner_point.header.seq = cloud_plane->header.seq;
    // estimated_corner_point.transform.translation = tf2::toMsg(position_corner_point);
    // estimated_corner_point.transform.rotation = tf2::toMsg(orientation_corner_point);

    // geometry_msgs::TransformStamped  pose_corner_transformed;
    // cloudBuffer.transform(estimated_corner_point, pose_corner_transformed,"base_footprint");

    // nav_msgs::Odometry pose_corner_odometry;
    // pose_corner_odometry.header = pose_corner_transformed.header;
    // pose_corner_odometry.header.frame_id = "base_footprint";
    // pose_corner_odometry.pose.pose.position.x = pose_corner_transformed.transform.translation.x;
    // pose_corner_odometry.pose.pose.position.y = pose_corner_transformed.transform.translation.y;
    // pose_corner_odometry.pose.pose.position.z = pose_corner_transformed.transform.translation.z;
    // //pose_corner_odometry.pose.pose.position.z = 0.37;
    // pose_corner_odometry.pose.pose.orientation = tf2::toMsg(orientation_corner_point);

    // ROS_INFO_STREAM("Corner: " << pose_corner_odometry);

    // pub_cloud_debug.publish(cloud_plane);
    
    // pub_pose_debug.publish(pose_corner_odometry);

    tf2_ros::Buffer cloudBuffer;
    tf2_ros::TransformListener tfListenerCloud(cloudBuffer);
    //ICP
    //std::vector<double> score_vec(28, 0.0);
    std::vector<double> score_vec;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > icp_vec;

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaximumIterations (1000000);
    icp.setTransformationEpsilon (1e-12);

    
    std::vector <nav_msgs::Odometry> pose_vec;


    int i=0;
    for (auto& clouds : clusters_vec)
    {
      icp.setInputSource (model_cloud);
      icp.setInputTarget (clouds);
      //icp.setInputSource (clouds);
      //icp.setInputTarget (model_cloud);
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

      //estimated_pose.header.frame_id = msg_cloud->header.frame_id;
      //estimated_pose.header.seq = msg_cloud->header.seq;
      estimated_pose.header.frame_id = msg_cloud->header.frame_id;
      estimated_pose.header.seq = msg_cloud->header.seq;
      estimated_pose.transform.translation = tf2::toMsg(position_cube);
      estimated_pose.transform.rotation = tf2::toMsg(cube_orientation);

      geometry_msgs::TransformStamped  pose_transformed;
      cloudBuffer.transform(estimated_pose, pose_transformed,"base_footprint",ros::Duration(0.5));

      // get the rotation matrix from the quaternion of pose_transformed
      tf2::Quaternion q(pose_transformed.transform.rotation.x, pose_transformed.transform.rotation.y, pose_transformed.transform.rotation.z, pose_transformed.transform.rotation.w);
      // quaternion to rotation matrix
      tf2::Matrix3x3 m(q);
      // correct the rotation matrix
      tf2::Matrix3x3 m2 = CorrectRotationMatrix(m);

      //for (int i = 0; i < 3; i++) {
      //  for (int j = 0; j < 3; j++) {
      
      //      std::cout << m2[i][j] << ' ';
      //  }
       
      // std::cout << std::endl;
      //}

      m2.getRotation(cube_orientation);


      nav_msgs::Odometry pose_odometry;
      pose_odometry.header = pose_transformed.header;
      pose_odometry.header.frame_id = "base_footprint";
      //pose_odometry.child_frame_id = pose_transformed.child_frame_id;
      pose_odometry.pose.pose.position.x = pose_transformed.transform.translation.x;
      pose_odometry.pose.pose.position.y = pose_transformed.transform.translation.y;
      pose_odometry.pose.pose.position.z = pose_transformed.transform.translation.z;
      pose_odometry.pose.pose.orientation = tf2::toMsg(cube_orientation);

      //std::cout << " x: " << pose_odometry.pose.pose.position.x << std::endl;
      //std::cout << " y: " << pose_odometry.pose.pose.position.y << std::endl;
      //std::cout << " z: " << pose_odometry.pose.pose.position.z << std::endl;


      //estimated_pose_vec.at(i) = pose_odometry;

      pose_vec.push_back(pose_odometry);

      i++;
    }

    int minElementIndex = std::max_element(score_vec.begin(),score_vec.end()) - score_vec.begin();
    //std::cout << "minElementIndex:" << minElementIndex << std::endl;
    
    pub_pose_debug.publish(pose_vec.at(minElementIndex));
    pub_cloud_debug.publish(clusters_vec.at(minElementIndex));

    //for(int i=0; i<pose_vec.size();++i){
    //  pose_publishers[i].publish(pose_vec[i]);
    //}

}

