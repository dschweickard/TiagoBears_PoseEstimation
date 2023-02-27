#include<TiagoBears_PoseEstimation/pose_estimator.h>
#include<TiagoBears_PoseEstimation/PoseEstimation.h>

PoseEstimator::PoseEstimator(ros::NodeHandle n){
    ROS_INFO("I heard: [constructor]");
    // initialize the publishers
    // for(int i=0; i<28; i++){
    //      char* topic;
    //      sprintf(topic, "/cube_%d_odom", i);
    //      pose_publishers.push_back(n.advertise<nav_msgs::Odometry>(topic,1));
    // }


    //pcl::io::loadPCDFile ("/meshs/cubeModel.pcd", blob);
    //ROS_INFO("I heard: [readPCD]");
    //pcl::fromPCLPointCloud2 (blob, cloud);
    //ROS_INFO("I heard: [Convert]");

    // initialize the point cloud subscriber
    //ROS_INFO("I heard: [Subscriber]");
    point_cloud_subscriber=n.subscribe("/xtion/depth_registered/points", 1, &PoseEstimator::pcl_callback, this);
    ROS_INFO("I heard: [Publisher1]");
    pub_cloud_debug = n.advertise<sensor_msgs::PointCloud2>("CloudFiltered", 1);
    ROS_INFO("I heard: [Publisher2]");
    pub_pose_debug = n.advertise<nav_msgs::Odometry>("CubePose", 1);
    ROS_INFO("I heard: [Publisher3]");
    pub_icp_debug = n.advertise<sensor_msgs::PointCloud2>("CloudICP", 1);
    //pose_server = n.advertiseService("PoseEstimation", &PoseEstimator::service_callback,this);

    //ROS_INFO("I heard: [Publisher 2]");
    //pub_cloud_cluster = n.advertise<sensor_msgs::PointCloud2>("cloudClusters", 1);

}



void checkFreeSpace ()
{

}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> EuclideanClustering (pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
{


}


std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> RegionGrowingClustering (pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
{
  pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (input_cloud);
  normal_estimator.setKSearch (50); //Best scale?
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::removeNaNFromPointCloud(*input_cloud, *indices);


  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (200);
  reg.setMaxClusterSize (1500);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (50);
  reg.setInputCloud (input_cloud);
  reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);
  std::vector <pcl::PointIndices> cluster_ind;
  reg.extract (cluster_ind);
  
  std::cout << "Number of clusters is equal to " << cluster_ind.size () << std::endl;
  std::cout << "First cluster has " << cluster_ind[0].indices.size () << " points." << std::endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  //pub_cloud_debug.publish(colored_cloud);


  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clust_vec;

  pcl::ExtractIndices<pcl::PointXYZ> extract_cluster;
    
  for (auto& cluster_idx : cluster_ind)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr idx (new pcl::PointIndices);
    *idx = cluster_idx;
    extract_cluster.setInputCloud(input_cloud);
    extract_cluster.setIndices(idx);
    extract_cluster.setNegative (false);
    extract_cluster.filter (*cloud_cluster);
    clust_vec.push_back(cloud_cluster);
  }

  return clust_vec;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ColorRegionGrowingClustering (pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
{


}


std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ConditionalEuclideanClustering (pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
{


}
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> SuperVoxelClustering (pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
{




  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clust_vec;

  pcl::ExtractIndices<pcl::PointXYZ> extract_cluster;
    
  // for (auto& cluster_idx : cluster_ind)
  // {
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  //   pcl::PointIndices::Ptr idx (new pcl::PointIndices);
  //   *idx = cluster_idx;
  //   extract_cluster.setInputCloud(input_cloud);
  //   extract_cluster.setIndices(idx);
  //   extract_cluster.setNegative (false);
  //   extract_cluster.filter (*cloud_cluster);
  //   clust_vec.push_back(cloud_cluster);
  // }

  return clust_vec;

}

// std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> CorrespondenceGrouping (pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
// {
//
//
// }


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


void PoseEstimator::pcl_callback(const pcl::PCLPointCloud2ConstPtr& msg_cloud){
    

    //boost::filesystem::path full_path(boost::filesystem::current_path());
    //std::cout << "Current path is : " << full_path << std::endl;

    //ROS_INFO("I heard: [Load Model]");
    ROS_INFO_STREAM("FrameID  " << msg_cloud->header.frame_id);
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile ("src/TiagoBears_PoseEstimation/src/cube.pcd", cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, *model_cloud); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>
    

    //pub_model_cloud.publish(cloud_blob);

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_filtered_cloud(new pcl::PointCloud< pcl::PointXYZ>);
   

    //Initialize new Pointers for cropped PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr msg_cloud_pcl(new pcl::PointCloud< pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filterd_vox(new pcl::PointCloud< pcl::PointXYZ>);

    // Convert from PointCloud2 to PointCloud
    pcl::fromPCLPointCloud2 (*msg_cloud, *msg_cloud_pcl);

    ROS_INFO_STREAM(":Size Input PointCloud  " << msg_cloud_pcl->size());

    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud (msg_cloud_pcl);
    vox.setLeafSize (0.003f, 0.003f, 0.003f);
    vox.filter (*cloud_filterd_vox);

    ROS_INFO_STREAM(":Size voxel filtered PointCloud  " << cloud_filterd_vox->size());

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
    

    ROS_INFO_STREAM(":Size PointCLoud of cubes  " << cloud_seg->size());
    

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cubes (new pcl::PointCloud<pcl::PointXYZ>(*cloud_seg));
    pub_icp_debug.publish(cloud_cubes);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    tree->setInputCloud (cloud_cubes);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
    ece.setClusterTolerance (0.003f); //cluster_tolerance 
    ece.setMinClusterSize (200); //cluster_min_size
    ece.setMaxClusterSize (1000); //cluster_max_size
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



    //clusters_vec = RegionGrowingClustering (cloud_cubes);
    ROS_INFO_STREAM("Size of pc2_clusters: " << clusters_vec.size());


    //sensor_msgs::PointCloud2::Ptr cluster_out(new sensor_msgs::PointCloud2);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_out(new pcl::PointCloud<pcl::PointXYZ>);
    //*cluster_out = *clusters.at(3);
    //ROS_INFO_STREAM("Size of pointcloud: " << cluster_out->size());
    
    //ROS_INFO_STREAM("Size of pc2_clusters: " << cluster_out.);


    
    //tf2_ros::Buffer tfBuffer;
    //tf2_ros::TransformListener tfListener(tfBuffer);
    //geometry_msgs::TransformStamped transformStamped;


    tf2_ros::Buffer cloudBuffer;
    geometry_msgs::TransformStamped transformStampedICPPose;
    tf2_ros::TransformListener tfListenerCloud(cloudBuffer);


    //ICP
    //std::vector<double> score_vec(28, 0.0);
    std::vector<double> score_vec;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > icp_vec;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations (100000);
    icp.setTransformationEpsilon (1e-12);
    // icp.setMaxCorrespondenceDistance (0.005);
    //icp.setRANSACOutlierRejectionThreshold (1.);
    
    std::vector <nav_msgs::Odometry> pose_vec;
    std::vector<nav_msgs::Odometry> estimated_pose_vec(28);

    int i=0;
    for (auto& clouds : clusters_vec)
    {
      icp.setInputSource (model_cloud);
      icp.setInputTarget (clouds);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud< pcl::PointXYZ>);
      const Eigen::Matrix4f init_guess;
      // init_guess << 0., 0.,0.,0.,
      //               0.,0.,0.,0.,
      //               0.,0.,0.,0.,
      //               0.,0.,0.,0.;
      //icp.computeTransformation(*cloud_icp, init_guess);
      icp.align (*cloud_icp);
      icp_vec.push_back(cloud_icp);
      double score = icp.getFitnessScore();
      std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
      Eigen::Matrix4f icp_transformation = icp.getFinalTransformation ();
      score_vec.push_back(score);
      //std::cout << " Prob: " << prob_vec.at(i) << std::endl;
      
      

      tf2::Vector3 position_cube = tf2::Vector3(icp_transformation(0,3),icp_transformation(1,3),icp_transformation(2,3));
      //Eigen::MatrixXf m(3,3);
      // Eigen::Matrix3f m;
      // m << icp_transformation(0,0),icp_transformation(0,1),icp_transformation(0,2),
      //                                                              icp_transformation(1,0),icp_transformation(1,1),icp_transformation(1,2),
      //                                                              icp_transformation(2,0),icp_transformation(2,1),icp_transformation(2,2);

      // std::cout << m << std::endl;

      // m = CorrectRotationMatrix(m);

      // std::cout << " Position of cube x: " << icp_transformation(0,3) << std::endl;
      // std::cout << "Position of cube y: " << icp_transformation(1,3) << std::endl;
      // std::cout << "Position of cube z: " << icp_transformation(2,3) << std::endl;


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
      cloudBuffer.transform(estimated_pose, pose_transformed,"base_footprint",ros::Duration(1.));

      // get the rotation matrix from the quaternion of pose_transformed
      tf2::Quaternion q(pose_transformed.transform.rotation.x, pose_transformed.transform.rotation.y, pose_transformed.transform.rotation.z, pose_transformed.transform.rotation.w);
      // quaternion to rotation matrix
      tf2::Matrix3x3 m(q);
      // correct the rotation matrix
      tf2::Matrix3x3 m2 = CorrectRotationMatrix(m);
      m2.getRotation(cube_orientation);


      // try{
      //   transformStamped = tfBuffer.lookupTransform("base_footprint","xtion_rgb_optical_frame",ros::Time(0),ros::Duration(0.1));
      // }
      // catch (tf2::TransformException &ex) {
      //    ROS_WARN("%s",ex.what());
      // }
      
      // Eigen::Matrix4d temp_matrix;
      // temp_matrix = icp_transformation.cast<double>();
      // Eigen::Affine3d pose_icp;
      // pose_icp.matrix() = temp_matrix;
      // //std::cout << " Matrix1: " << pose_icp.matrix() << std::endl;
      // geometry_msgs::TransformStamped estimated_pose = tf2::eigenToTransform(pose_icp);
      // //std::cout << " Matrix2: " << estimated_pose << std::endl;

      // geometry_msgs::TransformStamped  pose_transformed;
      // //tf2::doTransform(estimated_pose, pose_transformed, transformStamped);
      // estimated_pose.header.frame_id = "xtion_rgb_optical_frame";
      // Eigen::Affine3d pose_icp_transformed;
      // //tf2::doTransform(pose_icp, pose_icp_transformed, transformStamped);

      // tfBuffer.transform(estimated_pose,pose_transformed,"base_footprint",ros::Duration(0.1));
      // //std::cout << " Matrix: " << pose_transformed.matrix() << std::endl;

      // pose_transformed=estimated_pose;
      nav_msgs::Odometry pose_odometry;
      pose_odometry.header = pose_transformed.header;
      pose_odometry.header.frame_id = "base_footprint";
      //pose_odometry.child_frame_id = pose_transformed.child_frame_id;
      pose_odometry.pose.pose.position.x = pose_transformed.transform.translation.x;
      pose_odometry.pose.pose.position.y = pose_transformed.transform.translation.y;
      pose_odometry.pose.pose.position.z = pose_transformed.transform.translation.z;
      pose_odometry.pose.pose.orientation = tf2::toMsg(cube_orientation);

      std::cout << " x: " << pose_odometry.pose.pose.position.x << std::endl;
      std::cout << " y: " << pose_odometry.pose.pose.position.y << std::endl;
      std::cout << " z: " << pose_odometry.pose.pose.position.z << std::endl;


      estimated_pose_vec.at(i) = pose_odometry;

      pose_vec.push_back(pose_odometry);
      //geometry_msgs::Pose pose_msg;
      //tf2::doTransform()
      i++;
    } 
    int minElementIndex = std::max_element(score_vec.begin(),score_vec.end()) - score_vec.begin();
    std::cout << "minElementIndex:" << minElementIndex << std::endl;
    pub_pose_debug.publish(pose_vec.at(minElementIndex));
    pub_cloud_debug.publish(clusters_vec.at(minElementIndex));
    

}

tf2::Matrix3x3 correctCubeRotation (tf2::Matrix3x3 &matrix){
  for(int i=0; i<3;i++){

  }




}


//}
// Create Service node
// Refine Clustering
// Probabilities
// Check free space
// 


