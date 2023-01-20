#include <ros/ros.h>
#include "TiagoBears_PoseEstimation/pose_estimator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TiagoBears_PoseEstimation");
    ros::NodeHandle nh;
    PoseEstimator pose_estimator(nh);
}