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
    ROS_INFO("I heard: [Subscriber]");
    point_cloud_subscriber=n.subscribe("/xtion/depth_registered/points", 1, &PoseEstimator::pcl_callback, this);
    ROS_INFO("I heard: [Publisher 1]");
    pub_cloud_debug = n.advertise<sensor_msgs::PointCloud2>("CloudFiltered", 1);
    //ROS_INFO("I heard: [Publisher 2]");
    //pub_cloud_cluster = n.advertise<sensor_msgs::PointCloud2>("CloudCluster", 1);

}



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
    
    pub_cloud_debug.publish(cloud_seg);

    ROS_INFO_STREAM(":Size PointCLoud of cubes  " << cloud_seg->size());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cubes (new pcl::PointCloud<pcl::PointXYZ>(*cloud_seg));

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    tree->setInputCloud (cloud_cubes);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
    ece.setClusterTolerance (0.005f); //cluster_tolerance 
    ece.setMinClusterSize (200); //cluster_min_size
    ece.setMaxClusterSize (1200); //cluster_max_size
    ece.setSearchMethod (tree);
    ece.setInputCloud (cloud_cubes);
    ece.extract (cluster_indices);

    ROS_INFO_STREAM("Size of cluster_indices: " << cluster_indices.size());

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        cloud_cluster->points.push_back(cloud_cubes->points[*pit]);
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      //std::cout << "Cluster has " << cloud_cluster->points.size() << " points.\n";

    }
    //pub_cloud_cluster.publish(*(clusters.at(0)));
    //pub_cloud_debug.publish(*(clusters.at(0)));

    //ROS_INFO("[published Cluster]");
    // pcl::PCLPointCloud2 cloud_filtered;  
    // //pcl::PointCloud<PointCloud> unfiltered_cloud;
    // //pcl::fromROSMsg (msg_cloud, unfiltered_cloud);
    
    
    // //Create the filtering object
    // pcl::PCLPointCloud2::Ptr pcl_cloud;
        // //pcl_conversions::toPCLPtr(msg_cloud, pcl_cloud);


    //ICP
    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setMaximumIterations (1000);
    // icp.setInputSource (cloud_seg);
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



    // //Algorithm params
    // bool show_keypoints_ (false);
    // bool show_correspondences_ (false);
    // bool use_cloud_resolution_ (false);
    // bool use_hough_ (true);
    // float model_ss_ (0.01f);
    // float scene_ss_ (0.03f);
    // float rf_rad_ (0.015f);
    // float descr_rad_ (0.02f);
    // float cg_size_ (0.01f);
    // float cg_thresh_ (5.0f);

    // float resolution = static_cast<float> (computeCloudResolution (model_cloud));
    // if (resolution != 0.0f)
    // {
    //   model_ss_   *= resolution;
    //   scene_ss_   *= resolution;
    //   rf_rad_     *= resolution;
    //   descr_rad_  *= resolution;
    //   cg_size_    *= resolution;
    // }

    // std::cout << "Model resolution:       " << resolution << std::endl;
    // std::cout << "Model sampling size:    " << model_ss_ << std::endl;
    // std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
    // std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
    // std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
    // std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
    
    // pcl::PointCloud< pcl::Normal>::Ptr model_normals (new pcl::PointCloud< pcl::Normal> ());
    // pcl::PointCloud< pcl::Normal>::Ptr scene_normals (new pcl::PointCloud< pcl::Normal> ());
    // pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
    // norm_est.setKSearch (10);
    // norm_est.setInputCloud (model_cloud);
    // norm_est.compute (*model_normals);

    // norm_est.setInputCloud (input_filtered_cloud);
    // norm_est.compute (*scene_normals);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints (new pcl::PointCloud<pcl::PointXYZ> ());
    // pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZ> ());
    
    // pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    // uniform_sampling.setInputCloud (model_cloud);
    // uniform_sampling.setRadiusSearch (model_ss_);
    // uniform_sampling.filter (*model_keypoints);
    // std::cout << "Model total points: " << model_cloud->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

    // uniform_sampling.setInputCloud (input_filtered_cloud);
    // uniform_sampling.setRadiusSearch (scene_ss_);
    // uniform_sampling.filter (*scene_keypoints);
    // std::cout << "Scene total points: " << input_filtered_cloud->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;

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




