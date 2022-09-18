/**
 * @file alignment.cpp
 * @authors Arjun Datta (arjunvirdatta@gmail.com), Dyuman Aditya (dyuman.aditya@gmail.com)
 * @brief Source file for alignment. Contains Sample consensus prerejective and ICP
 * @version 1.0
 * @date 2021-07-10
 * 
 * @copyright Copyright (c) 2021 (we need licence)
 * 
 */


#include "alignment.h"

#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>



std::tuple<Eigen::Matrix4f, float> alignment::SampleConsensusPrerejective(pcl::PointCloud<pcl::PointNormal>::Ptr model_cloud, 
                                                                          pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_features,
                                                                          pcl::PointCloud<pcl::PointNormal>::Ptr scene_cloud, 
                                                                          pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features,
                                                                          int max_iterations, 
                                                                          int num_samples, 
                                                                          int correspondence_randomness, 
                                                                          float similarity_threshold, 
                                                                          float max_correspondence_distance, 
                                                                          float inlier_fraction)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr aligned_model (new pcl::PointCloud<pcl::PointNormal>);
    pcl::SampleConsensusPrerejective<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> align;
    // pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kd_tree_source (new pcl::KdTreeFLANN<pcl::FPFHSignature33>);
    // pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kd_tree_target (new pcl::KdTreeFLANN<pcl::FPFHSignature33>);

    // kd_tree_target->setInputCloud(scene_features); 
    // kd_tree_source->setInputCloud(model_features);
    // align.setSearchMethodSource(kd_tree_source);
    // align.setSearchMethodTarget(kd_tree_target);
   
    align.setInputSource(model_cloud);
    align.setSourceFeatures(model_features);
    align.setInputTarget(scene_cloud);
    align.setTargetFeatures(scene_features);
    align.setMaximumIterations(max_iterations);
    align.setNumberOfSamples(num_samples);
    align.setCorrespondenceRandomness(correspondence_randomness);
    align.setSimilarityThreshold(similarity_threshold);
    align.setMaxCorrespondenceDistance(max_correspondence_distance);
    align.setInlierFraction(inlier_fraction);
    align.align(*aligned_model);
    
    float score =  align.getFitnessScore();
    Eigen::Matrix4f transformation = align.getFinalTransformation();
    
    return {transformation, score};
}


std::tuple<Eigen::Matrix4f, float> alignment::vanillaICP(pcl::PointCloud<pcl::PointNormal>::Ptr model_cloud, 
                                                         pcl::PointCloud<pcl::PointNormal>::Ptr scene_cloud,
                                                         pcl::search::KdTree<pcl::PointNormal>::Ptr model_kd_tree,
                                                         int max_iterations,
                                                         float transformation_eps,
                                                         float max_correspondence_distance,
                                                         float fitness_eps,
                                                         float outlier_threshold)
{
    pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
    pcl::PointCloud<pcl::PointNormal> final;
    pcl::search::KdTree<pcl::PointNormal>::Ptr kd_tree_target (new pcl::search::KdTree<pcl::PointNormal>);
    
    kd_tree_target->setInputCloud(scene_cloud); 

    icp.setSearchMethodSource(model_kd_tree);
    icp.setSearchMethodTarget(kd_tree_target);
    
    icp.setInputTarget(scene_cloud); 
    icp.setInputSource(model_cloud);
    
    icp.setMaximumIterations (max_iterations);
    icp.setTransformationEpsilon (transformation_eps);
    icp.setMaxCorrespondenceDistance (max_correspondence_distance);
    icp.setEuclideanFitnessEpsilon (fitness_eps);
    icp.setRANSACOutlierRejectionThreshold (outlier_threshold);
    icp.align(final);
    
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    float score = icp.getFitnessScore();
    
    return {transformation, score};
}

std::tuple<Eigen::Matrix4f, float> alignment::runICP(pcl::PointCloud<pcl::PointNormal>::Ptr model_cloud,
                                                     pcl::PointCloud<pcl::PointNormal>::Ptr scene_cloud,
                                                     Eigen::Matrix4f transformation,
                                                     pcl::search::KdTree<pcl::PointNormal>::Ptr model_kd_tree,
                                                     int max_iterations,
                                                     float transformation_eps,
                                                     float max_correspondence_distance,
                                                     float fitness_eps,
                                                     float outlier_threshold)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr transformed_model_cloud (new pcl::PointCloud<pcl::PointNormal> ());   
    pcl::transformPointCloud(*model_cloud, *transformed_model_cloud, transformation);
    auto[icp_transformation_desc, icp_fitness_score_desc] = alignment::vanillaICP(transformed_model_cloud,
                                                                                  scene_cloud,
                                                                                  model_kd_tree,
                                                                                  max_iterations,
                                                                                  transformation_eps,
                                                                                  max_correspondence_distance,
                                                                                  fitness_eps,
                                                                                  outlier_threshold);
    return {icp_transformation_desc, icp_fitness_score_desc};
}