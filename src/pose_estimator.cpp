#include<TiagoBears_PoseEstimation/pose_estimator.h>


PoseEstimator::PoseEstimator(ros::NodeHandle n){
    ROS_INFO("I heard: [constructor]");
    // initialize the publishers
    for(int i=0; i<28; i++){
        char* topic;
        sprintf(topic, "/cube_%d_odom", i);
        pose_publishers.push_back(n.advertise<nav_msgs::Odometry>(topic,1));
    }

    //pcl::PointCloud<pcl::PointXYZ> cloud;
    //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/meshs/cubeModel.pcd", cloud) == -1) //* load the file
    //{
    //    ROS_INFO("Error reading pcd file");
    //    
    //}

    //pcl::io::loadPCDFile ("/meshs/cubeModel.pcd", blob);
    //ROS_INFO("I heard: [readPCD]");
    //pcl::fromPCLPointCloud2 (blob, cloud);
    //ROS_INFO("I heard: [Convert]");
    // initialize the point cloud subscriber
    point_cloud_subscriber=n.subscribe("/xtion/depth_registered/points", 1, &PoseEstimator::pcl_callback, this);

    pub_cloud_debug = n.advertise<sensor_msgs::PointCloud2>("filteredCloud", 1);
    
    //pub_model_cloud = n.advertise<sensor_msgs::PointCloud2>("modelCloud", 1);

}


double
computeCloudResolution (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<pcl::PointXYZ> tree;
  tree.setInputCloud (cloud);

  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    if (! std::isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}


void PoseEstimator::pcl_callback(const pcl::PCLPointCloud2ConstPtr& msg_cloud){
    

    //boost::filesystem::path full_path(boost::filesystem::current_path());
    //std::cout << "Current path is : " << full_path << std::endl;


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
    pcl::PCLPointCloud2 cloud_filtered;
    //pcl::PointCloud<PointCloud> unfiltered_cloud;
    //pcl::fromROSMsg (msg_cloud, unfiltered_cloud);
    
    
    //Create the filtering object
    pcl::PCLPointCloud2::Ptr pcl_cloud;
    pcl::VoxelGrid<pcl::PCLPointCloud2> vox;

    //pcl_conversions::toPCLPtr(msg_cloud, pcl_cloud);
    vox.setInputCloud (msg_cloud);
    
    vox.setLeafSize (0.01f, 0.01f, 0.01f);
    vox.filter (cloud_filtered);
    pub_cloud_debug.publish (cloud_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_filtered_cloud(new pcl::PointCloud< pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud_filtered, *input_filtered_cloud);

    
    pcl::CropBox<pcl::PointXYZ> crop;
    //pcl::CropBox<pcl::PCLPointCloud2> crop;
    crop.setInputCloud (input_filtered_cloud);
    crop.setMin(Eigen::Vector4f(-0.6, -0.1, +0.25, 0.));
    crop.setMax(Eigen::Vector4f(+0.6, +1.6, +2., 0.));
    //PointCloud::Ptr potential_ground_points (new PointCloud);
    crop.filter (*input_filtered_cloud);
    ROS_INFO("[filtered]");
    //pcl::PointCloud<pcl::PointXYZ>::Ptr input_filtered_cloud(new pcl::PointCloud< pcl::PointXYZ>);
    //pcl::fromPCLPointCloud2(cloud_filtered, *input_filtered_cloud);
    //pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    //pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    //pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    //seg.setOptimizeCoefficients (true);
    // Mandatory
    //seg.setModelType (pcl::SACMODEL_PLANE);
    //seg.setMethodType (pcl::SAC_RANSAC);
    //seg.setDistanceThreshold (0.01);

    //seg.setInputCloud (input_filtered_cloud);
    //seg.segment (*inliers, *coefficients);



    //ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations (100);
    icp.setInputSource (input_filtered_cloud);
    icp.setInputTarget (model_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud< pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ> cloud_icp;
    icp.align (*cloud_icp);


    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    
    Eigen::Matrix4f transformation = icp.getFinalTransformation ();
    std::cout << transformation << std::endl;
    
    // Executing the transformation
    //pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    //pcl::transformPointCloud (*model_cloud, *transformed_cloud, transformation);
    
    // Show alignment
    //pcl::visualization::PCLVisualizer visu("Alignment");
    //visu.addPointCloud (input_filtered_cloud, ColorHandlerT (input_filtered_cloud, 0.0, 255.0, 0.0), "scene");
    //visu.addPointCloud (cloud_icp, "object_aligned");
    //visu.spin ();
    //sensor_msgs::PointCloud2 debug_cloud;
    //pcl::toROSMsg(*model_cloud, debug_cloud);
    pub_cloud_debug.publish(*input_filtered_cloud);
    ROS_INFO("[published]");


    //Algorithm params
    bool show_keypoints_ (false);
    bool show_correspondences_ (false);
    bool use_cloud_resolution_ (false);
    bool use_hough_ (true);
    float model_ss_ (0.01f);
    float scene_ss_ (0.03f);
    float rf_rad_ (0.015f);
    float descr_rad_ (0.02f);
    float cg_size_ (0.01f);
    float cg_thresh_ (5.0f);

    float resolution = static_cast<float> (computeCloudResolution (model_cloud));
    if (resolution != 0.0f)
    {
      model_ss_   *= resolution;
      scene_ss_   *= resolution;
      rf_rad_     *= resolution;
      descr_rad_  *= resolution;
      cg_size_    *= resolution;
    }

    std::cout << "Model resolution:       " << resolution << std::endl;
    std::cout << "Model sampling size:    " << model_ss_ << std::endl;
    std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
    std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
    std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
    std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
    
    pcl::PointCloud< pcl::Normal>::Ptr model_normals (new pcl::PointCloud< pcl::Normal> ());
    pcl::PointCloud< pcl::Normal>::Ptr scene_normals (new pcl::PointCloud< pcl::Normal> ());
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setKSearch (10);
    norm_est.setInputCloud (model_cloud);
    norm_est.compute (*model_normals);

    norm_est.setInputCloud (input_filtered_cloud);
    norm_est.compute (*scene_normals);

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZ> ());
    
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud (model_cloud);
    uniform_sampling.setRadiusSearch (model_ss_);
    uniform_sampling.filter (*model_keypoints);
    std::cout << "Model total points: " << model_cloud->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

    uniform_sampling.setInputCloud (input_filtered_cloud);
    uniform_sampling.setRadiusSearch (scene_ss_);
    uniform_sampling.filter (*scene_keypoints);
    std::cout << "Scene total points: " << input_filtered_cloud->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;

    // pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors (new pcl::PointCloud<pcl::SHOT352> ());
    // pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors (new pcl::PointCloud<pcl::SHOT352> ());

    //   //  Compute Descriptor for keypoints
    // //
    // pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> descr_est;
    // descr_est.setRadiusSearch (descr_rad_);

    // descr_est.setInputCloud (model_keypoints);
    // descr_est.setInputNormals (model_normals);
    // descr_est.setSearchSurface (model_cloud);
    // descr_est.compute (*model_descriptors);

    // descr_est.setInputCloud (scene_keypoints);
    // descr_est.setInputNormals (scene_normals);
    // descr_est.setSearchSurface (input_filtered_cloud);
    // descr_est.compute (*scene_descriptors);

    //   //  Find Model-Scene Correspondences with KdTree
    // //
    // pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

    // pcl::KdTreeFLANN<pcl::SHOT352> match_search;
    // match_search.setInputCloud (model_descriptors);

    // //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    // for (std::size_t i = 0; i < scene_descriptors->size (); ++i)
    //     {
    //     std::vector<int> neigh_indices (1);
    //     std::vector<float> neigh_sqr_dists (1);
    //     if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
    //     {
    //     continue;
    //     }
    //     int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    //     if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    //     {
    //     pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
    //     model_scene_corrs->push_back (corr);
    //     }
    // }
    // std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;
}




