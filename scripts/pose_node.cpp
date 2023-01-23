#include <ros/ros.h>
#include "TiagoBears_PoseEstimation/pose_estimator.h"

int main(int argc, char **argv)
{
    ROS_INFO("I heard: [node]");
    ros::init(argc, argv, "TiagoBears_PoseEstimation");
    ROS_INFO("I heard: [node started]");
    ros::NodeHandle nh;
    PoseEstimator pose_estimator(nh);
}