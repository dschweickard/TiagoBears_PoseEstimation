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
    
    pose_server = n.advertiseService("PoseEstimation", &PoseEstimator::service_callback,this);

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
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> NormalBasedSegmentation (pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
{


}

// std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> CorrespondenceGrouping (pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
// {
//
//
// }





void PoseEstimator::pcl_callback(const pcl::PCLPointCloud2ConstPtr& msg_cloud){
    

    //boost::filesystem::path full_path(boost::filesystem::current_path());
    //std::cout << "Current path is : " << full_path << std::endl;

    //ROS_INFO("I heard: [Load Model]");

    pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile ("cube.pcd", cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, *model_cloud); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>
    
    //pub_model_cloud.publish(cloud_blob);

    //printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    //BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
    //   printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    //}
    //const pcl::PointCloud<PointCloud>::Ptr temp_cloud(new pcl::PointCloud<PointCloud>);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_filtered_cloud(new pcl::PointCloud< pcl::PointXYZ>);
    //pcl::fromPCLPointCloud2(cloud_filtered, *input_filtered_cloud);
   
    //pcl::PCLPointCloud2::Ptr cloud_cropped (new pcl::PCLPointCloud2);

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

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped_z(new pcl::PointCloud<pcl::PointXYZ>());
    // CropBox seems to not work for z dimension
    // pcl::PassThrough<pcl::PointXYZ> pass_z;
    // pass_z.setInputCloud(cloud_cropped);
    // pass_z.setFilterFieldName("z");
    // pass_z.setFilterLimits(0.1, 1.2);
    // pass_z.filter(*cloud_cropped_z);
    


  
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




    // pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    // pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    // normal_estimator.setSearchMethod (tree);
    // normal_estimator.setInputCloud (cloud_cubes);
    // normal_estimator.setKSearch (100); //Best scale? //50
    // normal_estimator.compute (*normals);

    // pcl::IndicesPtr indices (new std::vector <int>);
    // pcl::removeNaNFromPointCloud(*cloud_cubes, *indices);

    // // Benötigt fine tuning
    // pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    // reg.setMinClusterSize (200);
    // reg.setMaxClusterSize (1500);
    // reg.setSearchMethod (tree);
    // reg.setNumberOfNeighbours (50);
    // reg.setInputCloud (cloud_cubes);  
    // reg.setIndices (indices);
    // reg.setInputNormals (normals);
    // reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    // reg.setCurvatureThreshold (1.0);
    // std::vector <pcl::PointIndices> cluster_ind;
    // reg.extract (cluster_ind);
    
    // std::cout << "Number of clusters is equal to " << cluster_ind.size () << std::endl;
    // std::cout << "First cluster has " << cluster_ind[0].indices.size () << " points." << std::endl;

    // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    // ROS_INFO_STREAM(":Size ColoredPointCLoud of cubes  " << colored_cloud->size());

    // colored_cloud->header.frame_id = "base_footprint";
    // //colored_cloud->header.stamp = ros::Time::now();
    // sensor_msgs::PointCloud2 object_msg;
    // pcl::toROSMsg(*colored_cloud,object_msg);
    // pub_cloud_debug.publish(object_msg);


    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_vec;

    // pcl::ExtractIndices<pcl::PointXYZ> extract_cluster;
      
    // for (auto& cluster_idx : cluster_ind)
    // {
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    //   pcl::PointIndices::Ptr idx (new pcl::PointIndices);
    //   *idx = cluster_idx;
    //   extract_cluster.setInputCloud(cloud_cubes);
    //   extract_cluster.setIndices(idx);
    //   extract_cluster.setNegative (false);
    //   extract_cluster.filter (*cloud_cluster);
    //   clusters_vec.push_back(cloud_cluster);
    // }

    //clusters_vec = RegionGrowingClustering (cloud_cubes);
    ROS_INFO_STREAM("Size of pc2_clusters: " << clusters_vec.size());

    //pub_cloud_debug.publish(clusters_vec.at(1));

    //sensor_msgs::PointCloud2::Ptr cluster_out(new sensor_msgs::PointCloud2);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_out(new pcl::PointCloud<pcl::PointXYZ>);
    //*cluster_out = *clusters.at(3);
    //ROS_INFO_STREAM("Size of pointcloud: " << cluster_out->size());
    
    //ROS_INFO_STREAM("Size of pc2_clusters: " << cluster_out.);
    // sensor_msgs::PointCloud2::Ptr tempROSMsg(new sensor_msgs::PointCloud2);
    // pcl::toROSMsg(*cluster_out, *tempROSMsg);
    

    //pub_cloud_cluster.publish(*(clusters.at(0)));
    //pub_cloud_debug.publish(*(clusters.at(0)));

    //ROS_INFO("[published Cluster]");
    // pcl::PCLPointCloud2 cloud_filtered;  
    // //pcl::PointCloud<PointCloud> unfiltered_cloud;
    // //pcl::fromROSMsg (msg_cloud, unfiltered_cloud);
    
    
    // //Create the filtering object
    // pcl::PCLPointCloud2::Ptr pcl_cloud;
        // //pcl_conversions::toPCLPtr(msg_cloud, pcl_cloud);


    // tf::TransformListener tf_listener;
    // tf::StampedTransform cam_to_base;
    // tf_listener.lookupTransform("base_footprint",msg_cloud->header.frame_id,ros::Time(0),cam_to_base);
    // tf::Transform target_pose;
    
    
    
    std::vector <nav_msgs::Odometry> pose_vec;
    
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    
    //ICP
    //std::vector<double> score_vec(28, 0.0);
    std::vector<double> score_vec;
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

      pose_vec.push_back(pose_odometry);
      //geometry_msgs::Pose pose_msg;
      //tf2::doTransform()
      i++;
    } 
    int minElementIndex = std::min_element(score_vec.begin(),score_vec.end()) - score_vec.begin();
    std::cout << "minElementIndex:" << minElementIndex << std::endl;
    pub_pose_debug.publish(pose_vec.at(minElementIndex));
    pub_cloud_debug.publish(clusters_vec.at(minElementIndex));

    //pub_cloud_cluster.publish(icp_vec.at(0));
    
    //ROS_INFO("ICP done");

    // icp.setInputSource (cluster_out);
    // icp.setInputTarget (model_cloud);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud< pcl::PointXYZ>);
    // icp.align (*cloud_icp);
    // //ROS_INFO("ICP done");

    // std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    // icp.getFitnessScore() << std::endl;
    
    // Eigen::Matrix4f transformation = icp.getFinalTransformation ();
    // std::cout << transformation << std::endl;

    
    // Executing the transformation
    //pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    //pcl::transformPointCloud (*model_cloud, *transformed_cloud, transformation);
    
    // Show alignment
    //pcl::visualization::PCLVisualizer visu("Alignment");
    //visu.addPointCloud (input_filtered_cloud, "scene");
    //visu.addPointCloud (cloud_icp, "object_aligned");
    //visu.spin ();
    //sensor_msgs::PointCloud2 debug_cloud;
    //pcl::toROSMsg(*model_cloud, debug_cloud);

}

// Create Service node
// Refine Clustering
// Probabilities
// Check free space
// 
//
//


void service_callback(TiagoBears_PoseEstimation::PoseEstimation::Request& req, TiagoBears_PoseEstimation::PoseEstimation::Response& res)
{
  sensor_msgs::PointCloud2 msg_cloud;
  //pcl::PCLPointCloud2ConstPtr msg_cloud;
  sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/xtion/depth_registered/points", ros::Duration(10));
    if (msg == NULL)
        ROS_INFO("No point clound messages received");
    else
        msg_cloud = *msg;

//   pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::PCLPointCloud2 cloud_blob;
//   pcl::io::loadPCDFile ("cube.pcd", cloud_blob);
//   pcl::fromPCLPointCloud2 (cloud_blob, *model_cloud); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>
  

//   pcl::PointCloud<pcl::PointXYZ>::Ptr input_filtered_cloud(new pcl::PointCloud< pcl::PointXYZ>);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr msg_cloud_pcl(new pcl::PointCloud< pcl::PointXYZ>);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filterd_vox(new pcl::PointCloud< pcl::PointXYZ>);

//   // Convert from PointCloud2 to PointCloud
//   //pcl::fromPCLPointCloud2 (msg_cloud, *msg_cloud_pcl);
//   pcl::fromROSMsg(msg_cloud, *msg_cloud_pcl);

//   pcl::VoxelGrid<pcl::PointXYZ> vox;
//   vox.setInputCloud (msg_cloud_pcl);
//   vox.setLeafSize (0.003f, 0.003f, 0.003f);
//   vox.filter (*cloud_filterd_vox);


//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(new pcl::PointCloud< pcl::PointXYZ>);

//   // Set up CropBox and filter input PointCloud
//   pcl::CropBox<pcl::PointXYZ> crop;
//   crop.setInputCloud (cloud_filterd_vox);
//   // y and z parameter seem to crop same dimension just inverted?
//   crop.setMin(Eigen::Vector4f(-0.6, -0.6, -1, 0.));
//   crop.setMax(Eigen::Vector4f(+0.6, +1.1, +1, 0.));
//   crop.filter (*cloud_cropped);
//   ROS_INFO("[filtered]");


//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);

//   // Create the segmentation object for the planar model and set all the parameters
//   pcl::SACSegmentation<pcl::PointXYZ> seg;
//   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//   seg.setOptimizeCoefficients (true);
//   seg.setModelType (pcl::SACMODEL_PLANE);
//   seg.setMethodType (pcl::SAC_RANSAC);
//   seg.setMaxIterations (1000);
//   seg.setDistanceThreshold (0.005f);
//   // Segment largest planar component from cropped cloud
//   seg.setInputCloud (cloud_cropped);
//   seg.segment (*inliers, *coefficients);
//   if (inliers->indices.size () == 0)
//   {
//     ROS_INFO ("Could not estimate a planar model for the given dataset.") ;
//   }


//   // Extract planar inliers from input cloud
//   pcl::ExtractIndices<pcl::PointXYZ> extract;
//   extract.setInputCloud (cloud_cropped);
//   extract.setIndices(inliers);
//   extract.setNegative (false);

//   // Get points associated with planar surface
//   extract.filter (*cloud_plane);

//   // Remove the planar inliers and extract rest
//   extract.setNegative (true);
//   extract.filter (*cloud_seg);
  

//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cubes (new pcl::PointCloud<pcl::PointXYZ>(*cloud_seg));

//   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

//   tree->setInputCloud (cloud_cubes);

//   std::vector<pcl::PointIndices> cluster_indices;
//   pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
//   ece.setClusterTolerance (0.004f); //cluster_tolerance 
//   ece.setMinClusterSize (150); //cluster_min_size
//   ece.setMaxClusterSize (1500); //cluster_max_size
//   ece.setSearchMethod (tree);
//   ece.setInputCloud (cloud_cubes);
//   ece.extract (cluster_indices);
//   ROS_INFO_STREAM("Size of cluster_indices: " << cluster_indices.size());
  

//   std::vector<sensor_msgs::PointCloud2::Ptr> pc2_clusters;
//   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_vec;

//   pcl::ExtractIndices<pcl::PointXYZ> extract_cluster;
  
//   for (auto& cluster_idx : cluster_indices)
//   {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointIndices::Ptr idx (new pcl::PointIndices);
//     *idx = cluster_idx;
//     extract_cluster.setInputCloud(cloud_cubes);
//     extract_cluster.setIndices(idx);
//     extract_cluster.setNegative (false);
//     extract_cluster.filter (*cloud_cluster);
//     clusters_vec.push_back(cloud_cluster);
//   }
//   std::vector <nav_msgs::Odometry> pose_vec;
    
//   tf2_ros::Buffer tfBuffer;
//   tf2_ros::TransformListener tfListener(tfBuffer);
//   geometry_msgs::TransformStamped transformStamped;
  
//   //ICP
//   std::vector<double> score_vec(28, 0.0);
//   //std::vector<double> score_vec;
//   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > icp_vec;

//   pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//   icp.setMaximumIterations (1000);
//   icp.setTransformationEpsilon (1e-9);
//   //icp.setMaxCorrespondenceDistance (0.003);
//   //icp.setRANSACOutlierRejectionThreshold (1.);
  
//   std::vector<nav_msgs::Odometry> estimated_pose_vec(28);

//   int i=0;
//   for (auto& clouds : clusters_vec)
//   {
//     icp.setInputSource (clouds);
//     icp.setInputTarget (model_cloud);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud< pcl::PointXYZ>);

//     icp.align (*cloud_icp);
//     icp_vec.push_back(cloud_icp);
//     double score = icp.getFitnessScore();
//     std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
//     Eigen::Matrix4f icp_transformation = icp.getFinalTransformation ();
//     score_vec.push_back(score);
//     //std::cout << " Prob: " << prob_vec.at(i) << std::endl;
    

//     try{
//       transformStamped = tfBuffer.lookupTransform("base_footprint","xtion_depth_frame",ros::Time(0),ros::Duration(0.1));
//     }
//     catch (tf2::TransformException &ex) {
//         ROS_WARN("%s",ex.what());
//     }

//     Eigen::Matrix4d temp_matrix;
//     temp_matrix = icp_transformation.cast<double>();
//     Eigen::Affine3d pose_icp;
//     pose_icp.matrix() = temp_matrix;
//     //std::cout << " Matrix1: " << pose_icp.matrix() << std::endl;
//     geometry_msgs::TransformStamped estimated_pose = tf2::eigenToTransform(pose_icp);
//     //std::cout << " Matrix2: " << estimated_pose << std::endl;

//     geometry_msgs::TransformStamped  pose_transformed;
//     tf2::doTransform(estimated_pose, pose_transformed, transformStamped);
    
//     Eigen::Affine3d pose_icp_transformed;
//     tf2::doTransform(pose_icp, pose_icp_transformed, transformStamped);
//     std::cout << " Matrix: " << pose_icp_transformed.matrix() << std::endl;
    
//     nav_msgs::Odometry pose_odometry;

//     pose_odometry.header = pose_transformed.header;
//     pose_odometry.child_frame_id = pose_transformed.child_frame_id;
//     pose_odometry.pose.pose.position.x = pose_transformed.transform.translation.x;
//     pose_odometry.pose.pose.position.y = pose_transformed.transform.translation.y;
//     pose_odometry.pose.pose.position.z = pose_transformed.transform.translation.z;
//     pose_odometry.pose.pose.orientation = pose_transformed.transform.rotation;

//     std::cout << " x: " << pose_odometry.pose.pose.position.x << std::endl;
//     std::cout << " y: " << pose_odometry.pose.pose.position.y << std::endl;
//     std::cout << " z: " << pose_odometry.pose.pose.position.z << std::endl;
//     // tf::Transform tf_ref = tf::Transform(tf::Matrix3x3(icp_transformation(0,0),icp_transformation(0,1),icp_transformation(0,2),
//     //                                                             icp_transformation(1,0),icp_transformation(1,1),icp_transformation(1,2),
//     //                                                             icp_transformation(2,0),icp_transformation(2,1),icp_transformation(2,2)),
//     //                                               tf::Vector3(icp_transformation(0,3),icp_transformation(1,3),icp_transformation(2,3)));
//     // //tf::Vector3 trans = tf_ref.getOrigin();
//     // //tf::Quaternion rot = tf_ref.getRotation();
//     //target_pose = cam_to_base * tf_ref;
//     //geometry_msgs::Pose pos_tmp;
//     //tf::poseTFToMsg(target_pose, pos_tmp);


//     estimated_pose_vec.at(i) = pose_odometry;
    
//     res.poseArray[i] = pose_odometry;

    pose_vec.push_back(pose_odometry);
    //geometry_msgs::Pose pose_msg;
    //tf2::doTransform()
    i++;
  }

  //res = estimated_pose_vec;
}


