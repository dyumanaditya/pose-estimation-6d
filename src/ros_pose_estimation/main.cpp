/**
 * @file main.cpp
 * @authors Arjun Datta (arjunvirdatta@gmail.com), Dyuman Aditya (dyuman.aditya@gmail.com)
 * @brief Source file main that runs the pose estimation on an mkv video file or live kinect streaming (ROS based)
 * @version 1.0
 * @date 2021-07-10
 * 
 * @copyright Copyright (c) 2021 Telekinesis, Arjun Datta, Dyuman Aditya. All right reserved
 * This project is released under the MIT License.
 */

#include "ros_pose_estimation.h"

#include <ros/ros.h>



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pose_estimation_6d");
    ros::NodeHandle nh;
    ros::Rate rate(30); // frequency of operation
    ROSPoseEstimation ros_pose_estimator(nh);
    while(ros::ok)
    {
        ros::spin();
    }
    
    return 0;
}
