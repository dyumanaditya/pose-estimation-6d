/**
 * @file pose_estimation.h
 * @authors Arjun Datta (arjunvirdatta@gmail.com), Dyuman Aditya (dyuman.aditya@gmail.com)
 * @brief Header file for estimating the final pose
 * @version 1.0
 * @date 2021-07-10
 * 
 * @copyright Copyright (c) 2021 (we need licence)
 * 
 */


#ifndef POSE_ESTIMATION_H
#define POSE_ESTIMATION_H

#include "csv_parser.h"

#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh_omp.h>



/**
 * @brief Class that puts the entire 6D pose estimation pipeline together.
 * 
 */
class PoseEstimation
{
private:
    /**
     * @brief Path to the cad model (.obj file) whose pose is to be calculated
     */
    std::string cad_path;
    
    /**
     * @brief Name of the csv file in which poses will be written
     */
    std::string csv_name;

    /**
     * @brief If true prints extensive debugging statements
     */
    bool debug;

    /**
     * @brief Radius used to compute normals for scene point cloud
     */
    float compute_normals_radius;

    /**
     * @brief Radius used to compute descriptors for model
     */
    float descriptor_model_radius;

    /**
     * @brief Radius used to compute descriptors for scene
     */
    float descriptor_scene_radius;
    
    /**
     * @brief Length of the side of the box used in region segmentation
     */
    float segmentation_region_box_size;
    
    /**
     * @brief Minimum limit for passthrough filter along the x-axis
     */
    float segmentation_plane_filter_lim_xmin;
    
    /**
     * @brief Maximum limit for passthrough filter  along the x-axis
     */
    float segmentation_plane_filter_lim_xmax;
    
    /**
     * @brief Minimum limit for passthrough filter along the x-axis
     */
    float segmentation_plane_filter_lim_ymin;

    /**
     * @brief Maximum limit for passthrough filter along the x-axis
     */
    float segmentation_plane_filter_lim_ymax;

    /**
     * @brief Minimum limit for passthrough filter along the x-axis
     */
    float segmentation_plane_filter_lim_zmin;

    /**
     * @brief Maximum limit for passthrough filter along the x-axis
     */
    float segmentation_plane_filter_lim_zmax;

    /**
     * @brief Distance threshold for sac plane segmentation
     */
    float segmentation_plane_distance_threshold;
    
    /**
     * @brief Leaf size for voxel grid
     */
    float segmentation_voxel_down_leaf;
    
    /**
     * @brief Max RANSAC iterations
     */
    int alignment_max_iterations;
    
    /**
     * @brief Number of points to sample for generating/prerejecting a pose
     */
    int alignment_num_samples;
    
    /**
     * @brief Number of nearest features to use
     */
    float alignment_correspondence_randomness;
    
    /**
     * @brief Polygonal edge length similarity threshold
     */
    float alignment_similarity_threshold;
    
    /**
     * @brief Inlier threshold
     */
    float alignment_max_correspondence_distance;
    
    /**
     * @brief Required inlier fraction for accepting a pose hypothesis
     */
    float alignment_inlier_fraction;
    
    /**
     * @brief Max ICP iterations
     */
    int icp_max_iterations;
    
    /**
     * @brief Maximum allowable translation squared difference between two consecutive
     * transformations in order for an optimization to be considered as having converged to the final solution
     */
    float icp_transformation_eps;
    
    /**
     * @brief Maximum distance threshold between two correspondent points in source <-> target
     */
    float icp_max_correspondence_distance;
    
    /**
     * @brief Maximum allowed distance error before the algorithm will be considered to have converged
     */
    float icp_fitness_eps;
    
    /**
     * @brief Inlier distance threshold for the internal outlier rejection loop
     */
    float icp_outlier_threshold;

    /**
     * @brief How many frames the pipeline has processed
     */
    int frame_cnt;

    /**
     * @brief The coordinates describing the center of the object in the previous frame
     */
    Eigen::Vector3f prev_center;

    /**
     * @brief The point cloud of the .obj file. The format of the point cloud contains
     * XYZ and Normal data. This point cloud is stored because the model cloud is constant 
     * throughout the program
     */
	pcl::PointCloud<pcl::PointNormal>::Ptr model_cloud;

    /**
     * @brief The features of the model_cloud. The format of the cloud contains descriptor data.
     * This point cloud is stored because the model features are constant throughout the program
     */
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_features;

    /**
     * @brief
     */
    pcl::search::KdTree<pcl::PointNormal>::Ptr model_kd_tree;

    /**
     * @brief The 4x4 transformation matrix describing the previous frames'
     * aligned object transformation
     */
    Eigen::Matrix4f prev_transformation = Eigen::Matrix4f::Identity();

    /**
     * @brief Utility to visualize the aligned object in the scene
     * 
     * @param transformation The 4x4 transformation matrix that was returned from the alignment function
     * @param scene_cloud Ptr to the scene cloud
     */
    void visualizeTransformation(Eigen::Matrix4f transformation, pcl::PointCloud<pcl::PointNormal>::Ptr scene_cloud);

    double avg_time=0;
    double avg_score=0;

public:
    /**
     * @brief Construct a new Pose Estimation object
     * 
     * @param cad_path Path to the cad model (.obj file) whose pose is to be calculated
     * @param csv_name Name of the csv file in which poses will be written
     * @param debug Boolean to print more detailed debugging statements 
     * @param compute_normals_radius Radius used to compute normals for scene point cloud
     * @param descriptor_model_radius Radius used to compute descriptors for model
     * @param descriptor_scene_radius Radius used to compute descriptors for scene
     * @param segmentation_region_box_size Length of the side of the box used in region segmentation
     * @param segmentation_plane_filter_lim_xmin Minimum limit for passthrough filter along the x-axis
     * @param segmentation_plane_filter_lim_xmax Maximum limit for passthrough filter along the x-axis
     * @param segmentation_plane_filter_lim_ymin Minimum limit for passthrough filter along the y-axis
     * @param segmentation_plane_filter_lim_ymax Maximum limit for passthrough filter along the y-axis
     * @param segmentation_plane_filter_lim_zmin Minimum limit for passthrough filter along the z-axis
     * @param segmentation_plane_filter_lim_zmax Maximum limit for passthrough filter along the z-axis
     * @param segmentation_plane_distance_threshold Distance threshold for sac plane segmentation
     * @param segmentation_voxel_down_leaf Leaf size for voxel grid
     * @param alignment_max_iterations Max RANSAC iterations
     * @param alignment_num_samples Number of points to sample for generating/prerejecting a pose
     * @param alignment_correspondence_randomness Number of nearest features to use
     * @param alignment_similarity_threshold Polygonal edge length similarity threshold 
     * @param alignment_max_correspondence_distance Inlier threshold
     * @param alignment_inlier_fraction Required inlier fraction for accepting a pose hypothesis
     * @param icp_max_iterations Max ICP iterations
     * @param icp_transformation_eps Maximum allowable translation squared difference between two consecutive
     * transformations in order for an optimization to be considered as having converged to the final solution
     * @param icp_max_correspondence_distance Maximum distance threshold between two correspondent points in source <-> target
     * @param icp_fitness_eps Maximum allowed distance error before the algorithm will be considered to have converged
     * @param icp_outlier_threshold Inlier distance threshold for the internal outlier rejection loop
     */
    PoseEstimation(std::string cad_path,
                   std::string csv_name,
                   bool debug,
                   float compute_normals_radius,
                   float descriptor_model_radius,
                   float descriptor_scene_radius,
                   float segmentation_region_box_size,
                   float segmentation_plane_filter_lim_xmin,
                   float segmentation_plane_filter_lim_xmax,
                   float segmentation_plane_filter_lim_ymin,
                   float segmentation_plane_filter_lim_ymax,
                   float segmentation_plane_filter_lim_zmin,
                   float segmentation_plane_filter_lim_zmax,
                   float segmentation_plane_distance_threshold,
                   float segmentation_voxel_down_leaf,
                   int alignment_max_iterations,
                   int alignment_num_samples,
                   float alignment_correspondence_randomness,
                   float alignment_similarity_threshold,
                   float alignment_max_correspondence_distance,
                   float alignment_inlier_fraction,
                   int icp_max_iterations,
                   float icp_transformation_eps,
                   float icp_max_correspondence_distance,
                   float icp_fitness_eps,
                   float icp_outlier_threshold);

    /**
     * @brief Destroy the Pose Estimation object
     */
    ~PoseEstimation();

    /**
     * @brief Resposible for putting together the pipeline, detecting the object and returning it's pose in 6D space
     * 
     * @param cloud Scene cloud to detect object in
     * @return Eigen::Matrix4f 4x4 transformation matrix of the aligned object
     */
    Eigen::Matrix4f estimatePose(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    std::tuple<Eigen::Matrix4f, float, pcl::PointCloud<pcl::PointXYZ>::Ptr> trackingPose(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f prev_pos);

    std::tuple<Eigen::Matrix4f, float, pcl::PointCloud<pcl::PointXYZ>::Ptr> detectionPose(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool segment=0, Eigen::Matrix4f prev_pos = Eigen::Matrix4f::Identity());

    std::tuple<Eigen::Matrix4f, bool> compareResults(Eigen::Matrix4f tracking_pose, 
                                                                 float tracking_score, 
                                                                 Eigen::Matrix4f detection_pose, 
                                                                 float detection_score,
                                                                 Eigen::Matrix4f prev_kalman_pose,
                                                                 float time,
                                                                 float speed);

    /**
     * @brief Visualize the final trajectory of the object as calculated by the pose estimation pipeline
     * and display it on the scene point cloud
     * 
     * @param csv_path Path to the csv file containing all the pose information for each frame
     * @param pcd_path Path to the pcd file of the scene
     */
    void visualizeAllTransformation(std::string csv_path, std::string pcd_path);
};


#endif
