/**
 * @file ros_pose_estimation.cpp
 * @authors Arjun Datta (arjunvirdatta@gmail.com), Dyuman Aditya (dyuman.aditya@gmail.com)
 * @brief Source file Pose estimation ROS wrapper
 * @version 1.0
 * @date 2021-07-10
 * 
 * @copyright Copyright (c) 2021 (we need licence)
 * 
 */

#include "pose_estimation.h"
#include "csv_parser.h"
#include "ros_pose_estimation.h"
#include "pose_estimation_6d/UpdateKalmanFilterAndPredict.h"
#include "pose_estimation_6d/InitializeState.h"

#include <stdlib.h>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

#include <ctime>
#include <chrono>
#include <thread>
#include <future>

int TRACKER_COUNTER = 0;
int DETECTION_COUNTER = 0;

ROSPoseEstimation::ROSPoseEstimation(ros::NodeHandle &nh)
{
    // Get params from launch file
    // General params
    ros::param::get("~csv_name", csv_name);
    ros::param::get("~cad_path", cad_path);
    ros::param::get("~debug", debug);
    ros::param::get("~max_speed", max_speed);
    ros::param::get("~region_specific_detection", region_specific_detection);

    ros::param::get("~point_cloud_topic", point_cloud_topic);
    ros::param::get("~pose_pub_topic", pose_pub_topic);
    ros::param::get("~compute_normals_radius", compute_normals_radius);

    // Descriptor params
    ros::param::get("~descriptor_model_radius", descriptor_model_radius);
    ros::param::get("~descriptor_scene_radius", descriptor_scene_radius);

    // Segmentation params
    ros::param::get("~segmentation_region_box_size", segmentation_region_box_size);

    // Segmentation: Pass through filter
    ros::param::get("~segmentation_plane_filter_lim_xmin", segmentation_plane_filter_lim_xmin);
    ros::param::get("~segmentation_plane_filter_lim_xmax", segmentation_plane_filter_lim_xmax);
    ros::param::get("~segmentation_plane_filter_lim_ymin", segmentation_plane_filter_lim_ymin);
    ros::param::get("~segmentation_plane_filter_lim_ymax", segmentation_plane_filter_lim_ymax);
    ros::param::get("~segmentation_plane_filter_lim_zmin", segmentation_plane_filter_lim_zmin);
    ros::param::get("~segmentation_plane_filter_lim_zmax", segmentation_plane_filter_lim_zmax);

    ros::param::get("~segmentation_plane_distance_threshold", segmentation_plane_distance_threshold);
    ros::param::get("~segmentation_voxel_down_leaf", segmentation_voxel_down_leaf);

    // Alignment params
    ros::param::get("~alignment_max_iterations", alignment_max_iterations);
    ros::param::get("~alignment_num_samples", alignment_num_samples);
    ros::param::get("~alignment_correspondence_randomness", alignment_correspondence_randomness);
    ros::param::get("~alignment_similarity_threshold", alignment_similarity_threshold);
    ros::param::get("~alignment_max_correspondence_distance", alignment_max_correspondence_distance);
    ros::param::get("~alignment_inlier_fraction", alignment_inlier_fraction);
    ros::param::get("~icp_max_iterations", icp_max_iterations);
    ros::param::get("~icp_transformation_eps", icp_transformation_eps);
    ros::param::get("~icp_max_correspondence_distance", icp_max_correspondence_distance);
    ros::param::get("~icp_fitness_eps", icp_fitness_eps);
    ros::param::get("~icp_outlier_threshold", icp_outlier_threshold);

    // Pose estimation (non-ROS) object
    pose_estimator = new PoseEstimation(cad_path,
                                        csv_name,
                                        debug,
                                        compute_normals_radius,
                                        descriptor_model_radius,
                                        descriptor_scene_radius,
                                        segmentation_region_box_size,
                                        segmentation_plane_filter_lim_xmin,
                                        segmentation_plane_filter_lim_xmax,
                                        segmentation_plane_filter_lim_ymin,
                                        segmentation_plane_filter_lim_ymax,
                                        segmentation_plane_filter_lim_zmin,
                                        segmentation_plane_filter_lim_zmax,
                                        segmentation_plane_distance_threshold,
                                        segmentation_voxel_down_leaf,
                                        alignment_max_iterations,
                                        alignment_num_samples,
                                        alignment_correspondence_randomness,
                                        alignment_similarity_threshold,
                                        alignment_max_correspondence_distance,
                                        alignment_inlier_fraction,
                                        icp_max_iterations,
                                        icp_transformation_eps,
                                        icp_max_correspondence_distance,
                                        icp_fitness_eps,
                                        icp_outlier_threshold);

    // Subscribers
    point_cloud_sub = nh.subscribe(point_cloud_topic, 1, &ROSPoseEstimation::estimatePoseCallback, this);
    csv_parser = new CSVParser(ros::package::getPath("pose_estimation_6d") + "/csv_output", csv_name);
    // Publisher
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>(pose_pub_topic, 1);
    // Segmented Point Cloud publisher
    seg_cloud_pub = nh.advertise<PCLCloud>("seg_cloud", 1);

    // Kalman filter service
    kalman_filter_client = nh.serviceClient<pose_estimation_6d::UpdateKalmanFilterAndPredict>("update_kalman_filter_and_predict");
    kalman_initializer_client = nh.serviceClient<pose_estimation_6d::InitializeState>("initialize_kalman_state");

}

ROSPoseEstimation::~ROSPoseEstimation()
{
    delete pose_estimator;
    delete csv_parser;
}

void ROSPoseEstimation::estimatePoseCallback(const sensor_msgs::PointCloud2::ConstPtr &input_cloud)
{
    clock_t  start = clock();

    // Convert cloud2 to pcl format
    if (debug)
    {
        std::cout << "[ROSPoseEstimation::estimatePoseCallback] Starting Callback" << std::endl;
        std::cout << "[ROSPoseEstimation::estimatePoseCallback] Starting Conversion from PointCloud2 to PointXYZ" << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_cloud, *cloud);
    std::cout << "[ROSPoseEstimation::estimatePoseCallback] Conversion time: " << float(clock() - start) / CLOCKS_PER_SEC << std::endl;

    if (debug)
    {
        std::cout << "[ROSPoseEstimation::estimatePoseCallback] Conversion from PointCloud2 to PointXYZ Completed"<< std::endl;
    }
    // Run pose estimation
    if (first)
    {
        if (debug)
        {
            std::cout << "[ROSPoseEstimation::estimatePoseCallback] Starting Initial Dectection"<< std::endl;
        }
        
        ROSPoseEstimation::detection(cloud, Eigen::Matrix4f::Identity(), 100);
        
        if (debug)
        {
            std::cout << "[ROSPoseEstimation::estimatePoseCallback] Initial Dectection Completed"<< std::endl;
        }
    }

    std::cout<< "[ROSPoseEstimation::estimatePoseCallback] Starting Tracking"<< std::endl;
    auto [tracking_transformation, tracking_score, seg_cloud] = pose_estimator->trackingPose(cloud, prev_pos);
    if (tracking_score == -1.0f)
    {
        std::cout << "[ROSPoseEstimation::estimatePoseCallback] Tracking failed. Publishing previous pose." << std::endl;
    }
    prev_pos = tracking_transformation; //add something potential conflict
    std::cout << "[ROSPoseEstimation::estimatePoseCallback] Tracking Completed, score: "<<tracking_score<<std::endl;
    std::cout << "[ROSPoseEstimation::estimatePoseCallback] Number of running threads: " << detection_running << std::endl;
    TRACKER_COUNTER ++;
    if (detection_running == 0)
    {   
        DETECTION_COUNTER++;
        std::cout<< "[ROSPoseEstimation::estimatePoseCallback] Spawning Detection Thread"<<std::endl;
        std::thread detection_thread(&ROSPoseEstimation::detection, this, cloud, tracking_transformation, tracking_score);
        detection_thread.detach();
    }

    Eigen::Affine3f final_transformation;
    Eigen::Quaternionf quaternion;
    Eigen::Vector3f translation;
    final_transformation.matrix() = tracking_transformation;
    quaternion = final_transformation.rotation();
    translation = final_transformation.translation();

    csv_parser->writePoseToCsv(tracking_transformation);
    
    // Publish pose
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "depth_camera_link";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = translation[0];
    pose.pose.position.y = translation[1];
    pose.pose.position.z = translation[2];
    pose.pose.orientation.x = quaternion.x();
    pose.pose.orientation.y = quaternion.y();
    pose.pose.orientation.z = quaternion.z();
    pose.pose.orientation.w = quaternion.w();
    std::cout << "[ROSPoseEstimation::estimatePoseCallback] Publishing: x=" << pose.pose.position.x << ", y=" << pose.pose.position.y << ", z=" << pose.pose.position.z << std::endl;
    pose_pub.publish(pose);
    seg_cloud_pub.publish(seg_cloud);
    std::cout << "[ROSPoseEstimation::estimatePoseCallback] TRACKER_COUNTER: " << TRACKER_COUNTER << " DETECTION COUNTER: " << DETECTION_COUNTER << std::endl;
    if (debug)
    {
        std::cout << "[ROSPoseEstimation::estimatePoseCallback] Callback Completed.\n" << std::endl;
    }

}

void ROSPoseEstimation::detection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f tracking_transformation, float tracking_score)
{
    if (debug)
    {
        std::cout << "[ROSPoseEstimation::detection] Starting Detection" << std::endl;
    }
    
    clock_t start = clock();
    detection_running ++;
    Eigen::Matrix4f detection_transformation;
    float detection_score;
    pcl::PointCloud<pcl::PointXYZ>::Ptr seg_cloud;
    if (region_specific_detection)
    {   
        if (debug)
        {
            std::cout << "[ROSPoseEstimation::detection] Starting Full Frame and Region Specific Detection Asynchronously" << std::endl;
        }
        
        auto future_full = std::async(std::launch::async, &PoseEstimation::detectionPose, pose_estimator, cloud, 0, tracking_transformation);
        auto future_seg = std::async(std::launch::async, &PoseEstimation::detectionPose, pose_estimator, cloud, 1, tracking_transformation);
        auto [detection_seg_transformation, detection_seg_score, segmented_cloud] = future_seg.get();
        std::tie(detection_transformation, detection_score, seg_cloud) = future_full.get();
        
        if (debug)
        {
            std::cout << "[ROSPoseEstimation::detection] Asynchronous Full Frame and Region Specific Detection Completed" << std::endl;
        }
        
        if (detection_seg_score < detection_score)
        {   
            if (debug)
            {
                std::cout << "[ROSPoseEstimation::detection] Using  Region Specific Detection" << std::endl;
            }
            detection_transformation = detection_seg_transformation;
            detection_score = detection_seg_score;
        }
    }
    else
    {   
        if (debug)
        {
            std::cout << "[ROSPoseEstimation::detection] Starting Full Frame Detection" << std::endl;
        }
        
        std::tie(detection_transformation, detection_score, seg_cloud) = pose_estimator->detectionPose(cloud, 0, tracking_transformation);
        
        if (debug)
        {
            std::cout << "[ROSPoseEstimation::detection] Full Frame Detection Completed" << std::endl;
        }
    }

    clock_t middle = clock();
    float time = float(middle - start) / CLOCKS_PER_SEC; // time to check how probable results are

    float speed = max_speed;
    if (first)
    {
        Eigen::Affine3f t;
        Eigen::Vector3f translation;

        t.matrix() = detection_transformation;
        translation = t.translation();

        pose_estimation_6d::InitializeState srv;
        srv.request.x = translation[0]; // This is the x sent to KF for the update
        srv.request.y = translation[1]; // This is the y sent to KF for the update
        srv.request.z = translation[2]; // This is the z sent to KF for the update
        if (kalman_initializer_client.call(srv))
        {
            std::cout << "Kalman State Initialized" << std::endl;
        }
        else
        {
            ROS_ERROR_STREAM("Failed to call kalman filter service");
            ros::shutdown();
            exit(EXIT_FAILURE);
        }
        prev_kalman_pose = detection_transformation;
        std::cout << "Previous Kalman Pose: " << prev_kalman_pose << std::endl;

    }

    auto [transformation, correct_tracking] = pose_estimator->compareResults(tracking_transformation, tracking_score, detection_transformation, detection_score, prev_kalman_pose, time, speed);
    Eigen::Affine3f t;
    Eigen::Quaternionf quaternion;
    Eigen::Vector3f translation;
    t.matrix() = transformation;
    quaternion = t.rotation();
    translation = t.translation();

    if (debug)
    {
        std::cout << "[ROSPoseEstimation::detection] Starting Kalman Filter Computation" << std::endl;
    }
    pose_estimation_6d::UpdateKalmanFilterAndPredict srv;
    srv.request.x = translation[0]; // This is the x sent to KF for the update
    srv.request.y = translation[1]; // This is the y sent to KF for the update
    srv.request.z = translation[2]; // This is the z sent to KF for the update

    clock_t end = clock();
    std::cout << "[ROSPoseEstimation::detection] dt: " << float(end - start) / CLOCKS_PER_SEC << std::endl;
    srv.request.dt = float(end - start) / CLOCKS_PER_SEC; // This is the dt sent to KF for the predict

    Eigen::Vector3f kf_prediction; // This is for the kf prediction position
    if (kalman_filter_client.call(srv))
    {
        kf_prediction[0] = srv.response.x;
        kf_prediction[1] = srv.response.y;
        kf_prediction[2] = srv.response.z;
    }
    else
    {
        ROS_ERROR_STREAM("Failed to call kalman filter service");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }
    std::cout << "[ROSPoseEstimation::detection] Kalman Prediction: " << kf_prediction << " Translation: " << translation << std::endl;
    if (debug)
    {
        std::cout << "[ROSPoseEstimation::detection] Kalman Filter Computation Completed" << std::endl;
    }
    
    if (correct_tracking)
    {
        if (debug)
        {
            std::cout << "[ROSPoseEstimation::detection] Correcting Tracking" << std::endl;
        }
        prev_pos.block<3, 1>(0, 3) = kf_prediction;
    }
    else
    {
        std::cout << "[ERROR] FAILED TO UPDATE TRACKER." << std::endl;
    }

    prev_kalman_pose.block<3, 1>(0, 3) = kf_prediction;
    std::cout << "Previous Kalman Pose: " << prev_kalman_pose << std::endl;

    if (first)
    {
        prev_pos.block<3, 1>(0, 3) = translation;
        first = 0;
    }
    
    detection_running = 0;
    std::cout<< "[ROSPoseEstimation::detection] Detection Thread Completed in: "<< float(clock() - start) / CLOCKS_PER_SEC << std::endl;

}
