/**
 * @file alignment.h
 * @authors Arjun Datta (arjunvirdatta@gmail.com), Dyuman Aditya (dyuman.aditya@gmail.com)
 * @brief Alignment functions: Random sample consensus prerejective and ICP
 * @version 1.0
 * @date 2021-07-10
 * 
 * @copyright Copyright (c) 2021 (we need licence)
 * 
 */

#ifndef ALIGNMENT_H
#define ALIGNMENT_H

#include <pcl/features/fpfh_omp.h>



/**
 * @brief Namespace containing all the alignment functions
 * 
 */
namespace alignment
{
    /** 
     * @brief Align the model and scene point clouds and return the final transformation using sample consensus prerejective
     * 
     * @param model_cloud Pointer to point cloud belonging to model
     * @param model_features Pointer to point cloud of features of model_cloud
     * @param scene_cloud Pointer to point cloud belonging to scene
     * @param scene_features Pointer to point cloud of features of scene_cloud
     * 
     * @param max_iterations Max RANSAC iterations
     * @param num_samples Number of points to sample for generating/prerejecting a pose
     * @param correspondence_randomness Number of nearest features to use
     * @param similarity_threshold Polygonal edge length similarity threshold
     * @param max_correspondence_distance Inlier threshold
     * @param inlier_fraction Required inlier fraction for accepting a pose hypothesis
     * 
     * @return std::tuple<Eigen::Matrix4f, float>  4x4 transformation matrix (estimated pose)
     */
    std::tuple<Eigen::Matrix4f, float> SampleConsensusPrerejective(pcl::PointCloud<pcl::PointNormal>::Ptr model_cloud, 
                                                                   pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_features,
                                                                   pcl::PointCloud<pcl::PointNormal>::Ptr scene_cloud,
                                                                   pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features,
                                                                   int max_iterations, 
                                                                   int num_samples, 
                                                                   int correspondence_randomness, 
                                                                   float similarity_threshold, 
                                                                   float max_correspondence_distance, 
                                                                   float inlier_fraction); 

    /** 
     * @brief Align the model and scene point clouds and return the final transformation and score using vanilla ICP
     * 
     * @param model_cloud Pointer to point cloud belonging to model
     * @param scene_cloud Pointer to point cloud belonging to scene
     * 
     * @param max_iterations Max ICP iterations
     * @param transformation_eps Maximum allowable translation squared difference between two consecutive transformations in order for an optimization to be considered as having converged to the final solution
     * @param max_correspondence_distance Maximum distance threshold between two correspondent points in source <-> target
     * @param fitness_eps Maximum allowed distance error before the algorithm will be considered to have converged
     * @param outlier_threshold Inlier distance threshold for the internal outlier rejection loop
     * 
     * @return std::tuple<Eigen::Matrix4f, float> tuple where first value is an Eigen::Matrix4f 4x4 transformation matrix (estimated pose)and the second is a float that determines the fitness score
     */
    std::tuple<Eigen::Matrix4f, float> vanillaICP(pcl::PointCloud<pcl::PointNormal>::Ptr model_cloud, 
												  pcl::PointCloud<pcl::PointNormal>::Ptr scene_cloud,
                                                  pcl::search::KdTree<pcl::PointNormal>::Ptr model_kd_tree,
												  int max_iterations, 
                                                  float transformation_eps, 
                                                  float max_correspondence_distance, 
                                                  float fitness_eps, 
                                                  float outlier_threshold);

    /**
     * @brief Resposible for running ICP on the model cloud and scene cloud. Calls vanilla ICP
     * 
     * @param model_cloud Pointer to point cloud belonging to model
     * @param scene_cloud Pointer to point cloud belonging to scene
     * @param transformation Transformation matrix describing the pose of the object after the first alignment step
     * 
     * @param max_iterations Max ICP iterations
     * @param transformation_eps Maximum allowable translation squared difference between two consecutive transformations in order for an optimization to be considered as having converged to the final solution
     * @param max_correspondence_distance Maximum distance threshold between two correspondent points in source <-> target
     * @param fitness_eps Maximum allowed distance error before the algorithm will be considered to have converged
     * @param outlier_threshold Inlier distance threshold for the internal outlier rejection loop
     * 
     * @return std::tuple<Eigen::Matrix4f, float> tuple where first value is an Eigen::Matrix4f 4x4 transformation matrix (estimated pose)and the second is a float that determines the fitness score
     */
    std::tuple<Eigen::Matrix4f, float> runICP(pcl::PointCloud<pcl::PointNormal>::Ptr model_cloud,
                                              pcl::PointCloud<pcl::PointNormal>::Ptr scene_cloud,
                                              Eigen::Matrix4f transformation,
                                              pcl::search::KdTree<pcl::PointNormal>::Ptr model_kd_tree,
                                              int max_iterations,
                                              float transformation_eps,
                                              float max_correspondence_distance,
                                              float fitness_eps,
                                              float outlier_threshold);


}; // namespace alignment


#endif