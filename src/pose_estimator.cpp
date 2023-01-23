#include<TiagoBears_PoseEstimation/pose_estimator.h>


PoseEstimator::PoseEstimator(ros::NodeHandle n){
    ROS_INFO("I heard: [constructor]");
    // initialize the publishers
    for(int i=0; i<28; i++){
        char* topic;
        sprintf(topic, "/cube_%d_odom", i);
        pose_publishers.push_back(n.advertise<nav_msgs::Odometry>(topic,1));
    }
    // initialize the point cloud subscriber
    point_cloud_subscriber=n.subscribe<PointCloud>("/xtion/depth_registered/points", 1, &PoseEstimator::pcl_callback, this);
}

void PoseEstimator::pcl_callback(const PointCloud::ConstPtr& msg){
    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
        printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    }
}