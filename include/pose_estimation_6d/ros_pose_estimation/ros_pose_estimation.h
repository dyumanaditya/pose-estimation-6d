/**
 * @file ros_pose_estimation.h
 * @authors Arjun Datta (arjunvirdatta@gmail.com), Dyuman Aditya (dyuman.aditya@gmail.com)
 * @brief Pose estimation ROS wrapper
 * @version 1.0
 * @date 2021-07-10
 * 
 * @copyright Copyright (c) 2021 (we need licence)
 * 
 */

#include "pose_estimation.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>



class ROSPoseEstimation
{
private:
    // Parameters from launch file
    // General params
    std::string cad_path;
    std::string csv_name;
    std::string point_cloud_topic;
    std::string pose_pub_topic;
    bool debug;
    float max_speed;
    bool region_specific_detection;
    float compute_normals_radius;
    int detection_running=0;
    bool correct_tracking;
    bool first=1;
    Eigen::Matrix4f prev_pos = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f prev_kalman_pose;


    // Descriptor params
    float descriptor_model_radius;
    float descriptor_scene_radius;

    // Segmentation params
    float segmentation_region_box_size;
    // Segmentation: Pass through filter
    float segmentation_plane_filter_lim_xmin;
    float segmentation_plane_filter_lim_xmax;
    float segmentation_plane_filter_lim_ymin;
    float segmentation_plane_filter_lim_ymax;
    float segmentation_plane_filter_lim_zmin;
    float segmentation_plane_filter_lim_zmax;
    float segmentation_plane_distance_threshold;
    float segmentation_voxel_down_leaf;

    // Alignment params
    int alignment_max_iterations;
    int alignment_num_samples;
    float alignment_correspondence_randomness;
    float alignment_similarity_threshold;
    float alignment_max_correspondence_distance;
    float alignment_inlier_fraction;
    int icp_max_iterations;
    float icp_transformation_eps;
    float icp_max_correspondence_distance;
    float icp_fitness_eps;
    float icp_outlier_threshold;

    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> point_clouds;
    CSVParser* csv_parser;
    ros::Subscriber point_cloud_sub;
    // ros::Subscriber point_cloud_saver_sub;
    ros::Publisher pose_pub;
    ros::Publisher seg_cloud_pub;

    ros::ServiceClient kalman_filter_client;
    ros::ServiceClient kalman_initializer_client;

    PoseEstimation* pose_estimator;
    void detection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,  Eigen::Matrix4f tracking_transformation, float tracking_score);

public:
    ROSPoseEstimation(ros::NodeHandle& nh);
    ~ROSPoseEstimation();

    void estimatePoseCallback(const sensor_msgs::PointCloud2::ConstPtr &input_cloud);
    // void pointCloudCallback(const sensor_msgs::PointCloud2& input_cloud);
};
