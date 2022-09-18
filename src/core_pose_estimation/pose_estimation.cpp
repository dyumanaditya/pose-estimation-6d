/**
 * @file pose_estimation.cpp
 * @authors Arjun Datta (arjunvirdatta@gmail.com), Dyuman Aditya (dyuman.aditya@gmail.com)
 * @brief Source file pose estimation that contains the entire pipeline, and returns the pose of the object
 * @version 1.0
 * @date 2021-07-10
 * 
  * @copyright Copyright (c) 2021 Telekinesis, Arjun Datta, Dyuman Aditya. All right reserved
 *  This project is released under the MIT License.
 */


#include "pose_estimation.h"
#include "csv_parser.h"
#include "alignment.h"
#include "descriptor.h"
#include "segmentation.h"
#include "utils.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/visualization/common/impl/shapes.hpp>
#include <pcl/filters/radius_outlier_removal.h>

#include <thread>
#include <future>
#include <ctime>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdlib.h>



PoseEstimation::PoseEstimation(std::string cad_path,
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
                               float icp_outlier_threshold)
    : cad_path(cad_path),
      csv_name(csv_name),
      debug(debug),
      compute_normals_radius(compute_normals_radius),
      descriptor_model_radius(descriptor_model_radius),
      descriptor_scene_radius(descriptor_scene_radius),
      segmentation_region_box_size(segmentation_region_box_size),
      segmentation_plane_filter_lim_xmin(segmentation_plane_filter_lim_xmin),
      segmentation_plane_filter_lim_xmax(segmentation_plane_filter_lim_xmax),
      segmentation_plane_filter_lim_ymin(segmentation_plane_filter_lim_ymin),
      segmentation_plane_filter_lim_ymax(segmentation_plane_filter_lim_ymax),
      segmentation_plane_filter_lim_zmin(segmentation_plane_filter_lim_zmin),
      segmentation_plane_filter_lim_zmax(segmentation_plane_filter_lim_zmax),
      segmentation_plane_distance_threshold(segmentation_plane_distance_threshold),
      segmentation_voxel_down_leaf(segmentation_voxel_down_leaf),
      alignment_max_iterations(alignment_max_iterations),
      alignment_num_samples(alignment_num_samples),
      alignment_correspondence_randomness(alignment_correspondence_randomness),
      alignment_similarity_threshold(alignment_similarity_threshold),
      alignment_max_correspondence_distance(alignment_max_correspondence_distance),
      alignment_inlier_fraction(alignment_inlier_fraction),
      icp_max_iterations(icp_max_iterations),
      icp_transformation_eps(icp_transformation_eps),
      icp_max_correspondence_distance(icp_max_correspondence_distance),
      icp_fitness_eps(icp_fitness_eps),
      icp_outlier_threshold(icp_outlier_threshold),
      frame_cnt(0)
{   
    if(debug)
    {
        std::cout<<"[PoseEstimation::PoseEstimation] Starting Initialization"<<std::endl;
    }
    // Check for valid .obj path
    if (cad_path=="")
    {
        std::cout << "Please enter a valid .obj path while launching. ex: roslaunch pose_estimation pose_estimation.launch cad_path:=/home/user/model.obj" << std::endl;
        exit(EXIT_FAILURE);
    }
    // Compute object cloud and store object features which will stay the same throughout the program
    model_cloud = pcl::PointCloud<pcl::PointNormal>::Ptr (new pcl::PointCloud<pcl::PointNormal>());
    model_features = pcl::PointCloud<pcl::FPFHSignature33>::Ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
    
    if(debug)
    {
        std::cout<<"[PoseEstimation::PoseEstimation] Computing Normal Computation"<<std::endl;
    }
    
    model_cloud = utils::objToCloud(cad_path);
    
    if(debug)
    {
        std::cout<<"[PoseEstimation::PoseEstimation] Normal Computation Completed"<< std::endl;
        std::cout<<"[PoseEstimation::PoseEstimation] Starting Voxel Downsampling: "<< model_cloud->size()<<" points in cloud"<<std::endl;
    }
    
    model_cloud = segmentation::voxelDown<pcl::PointNormal>(model_cloud, segmentation_voxel_down_leaf);
    
    if(debug)
    {
        std::cout<<"[PoseEstimation::PoseEstimation] Voxel Downsampling Completed: "<< model_cloud->size()<<" points in cloud"<<std::endl;
        std::cout<<"[PoseEstimation::PoseEstimation] Starting Descriptor Computation"<<std::endl;
    }
    
    model_features = descriptor::computeDescriptors(model_cloud, descriptor_model_radius);
    
    if(debug)
    {
        std::cout<<"[PoseEstimation::PoseEstimation] Descriptor Computation Completed"<<std::endl;
        std::cout<<"[PoseEstimation::PoseEstimation] Staring KD-Tree Population"<<std::endl;
    }
    
    model_kd_tree = pcl::search::KdTree<pcl::PointNormal>::Ptr (new pcl::search::KdTree<pcl::PointNormal>);
    model_kd_tree->setInputCloud(model_cloud);  
    
    if(debug)
    {
        std::cout<<"[PoseEstimation::PoseEstimation] KD-Tree Population Completed"<<std::endl;
        std::cout<<"[PoseEstimation::PoseEstimation] Initialization Completed"<<std::endl;
    }
}

PoseEstimation::~PoseEstimation()
{
}

std::tuple<Eigen::Matrix4f, float, pcl::PointCloud<pcl::PointXYZ>::Ptr> PoseEstimation::trackingPose(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f prev_pos)
{
    if(debug)
    {
        std::cout<<"[PoseEstimation::trackingPose] Starting Tracking"<<std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr seg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr scene_cloud(new pcl::PointCloud<pcl::PointNormal>);
    Eigen::Affine3f prev_transformation;
    prev_transformation.matrix() = prev_pos; 
    prev_center = prev_transformation.translation();

    if(debug)
    {
        std::cout<<"[PoseEstimation::trackingPose] Starting Segmentation: "<< cloud->size()<<" points in cloud"<<std::endl;
        std::cout<<"[PoseEstimation::trackingPose] Starting Voxel Downsampling"<<std::endl;
    }
    // Voxel Downsampling
    cloud = segmentation::voxelDown<pcl::PointXYZ>(cloud, segmentation_voxel_down_leaf);
    
    if(debug)
    {
        std::cout<<"[PoseEstimation::trackingPose] Voxel Downsampling Completed: "<< cloud->size()<<" points in cloud"<<std::endl;
        std::cout<<"[PoseEstimation::trackingPose] Starting Plane Segmentation"<<std::endl;
    }
    // Plane segmentation [remove plane]
    cloud = segmentation::segPlane(cloud,
                                   segmentation_plane_filter_lim_xmin,
                                   segmentation_plane_filter_lim_xmax,
                                   segmentation_plane_filter_lim_ymin,
                                   segmentation_plane_filter_lim_ymax,
                                   segmentation_plane_filter_lim_zmin,
                                   segmentation_plane_filter_lim_zmax,
                                   segmentation_plane_distance_threshold);
    *seg_cloud = *cloud;

    if(debug)
    {
        std::cout<<"[PoseEstimation::trackingPose] Plane Segmentation Completed: "<< cloud->size()<<" points in cloud"<<std::endl;
        std::cout<<"[PoseEstimation::trackingPose] Starting Region Segmentation"<<std::endl;
    }
    // Region Segmentation [remove everything except region around prev_center]
    cloud = segmentation::segRegion(cloud, prev_center, segmentation_region_box_size);
    
    if(debug)
    {
        std::cout<<"[PoseEstimation::trackingPose] Region Segmentation Completed: "<< cloud->size()<<" points in cloud"<<std::endl;
        std::cout<<"[PoseEstimation::trackingPose] Ending Segmentation"<<std::endl;
    }
    if(cloud->size() == 0)  
    {    
        std::cout<<"[PoseEstimation::trackingPose] Error Tracking: empty cloud, relax segmentation parameters"<<std::endl;
        exit(1);
        return {prev_pos, -1.0f, seg_cloud};
    }

    if(debug)
    {
        std::cout<<"[PoseEstimation::trackingPose] Starting Normal Computation"<<std::endl;
    }
    // Compute normals of cloud
    scene_cloud = utils::computeCloudNormals(cloud, compute_normals_radius);
    
    if(debug)
    {
        std::cout<<"[PoseEstimation::trackingPose] Normal Computation Completed"<<std::endl;
        std::cout<<"[PoseEstimation::trackingPose] Starting ICP"<<std::endl;
    }
    // ICP 
    auto[tracking_transformation, tracking_fitness_score] = alignment::runICP( model_cloud, scene_cloud, prev_pos, model_kd_tree,
                                                                     icp_max_iterations, icp_transformation_eps,
                                                                     icp_max_correspondence_distance, icp_fitness_eps,
                                                                     icp_outlier_threshold);

    if(debug)
    {
        std::cout<<"[PoseEstimation::trackingPose] ICP Completed"<<std::endl;
    }
    
    Eigen::Matrix4f final_transformation =  tracking_transformation * prev_pos;
    prev_transformation = final_transformation; 
    
    if(debug)
    {
        std::cout<<"[PoseEstimation::trackingPose] Ending Tracking"<<std::endl;
    }
    
    return {final_transformation, tracking_fitness_score, cloud};
}

std::tuple<Eigen::Matrix4f, float, pcl::PointCloud<pcl::PointXYZ>::Ptr> PoseEstimation::detectionPose(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool segment, Eigen::Matrix4f inital_guess)
{
    if(debug)
    {
        std::cout<<"[PoseEstimation::detectionPose] Starting Detection"<<std::endl;
    }
    // Scene cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr scene_cloud(new pcl::PointCloud<pcl::PointNormal>);


    // Scene segmentation
    if(debug)
    {
        std::cout<<"[PoseEstimation::detectionPose] Starting Segmentation: "<< cloud->size()<<" points in cloud"<<std::endl;
        std::cout<<"[PoseEstimation::detectionPose] Starting Voxel Downsampling"<<std::endl;
    }
    // Voxel downsampling
    cloud = segmentation::voxelDown<pcl::PointXYZ>(cloud, segmentation_voxel_down_leaf);
    
    if(debug)
    {
        std::cout<<"[PoseEstimation::detectionPose] Voxel Downsampling Completed: "<< cloud->size()<<" points in cloud"<< std::endl;
        std::cout<<"[PoseEstimation::detectionPose] Starting Plane Segmentation"<<std::endl;
    }
    // Plane segmentation [remove plane]
    cloud = segmentation::segPlane(cloud,
                                   segmentation_plane_filter_lim_xmin,
                                   segmentation_plane_filter_lim_xmax,
                                   segmentation_plane_filter_lim_ymin,
                                   segmentation_plane_filter_lim_ymax,
                                   segmentation_plane_filter_lim_zmin,
                                   segmentation_plane_filter_lim_zmax,
                                   segmentation_plane_distance_threshold);
    
    if(debug)
    {
        std::cout<<"[PoseEstimation::detectionPose] Plane Segmentation Completed: "<< cloud->size() <<" points in cloud"<< std::endl;
    }
    // Region Segmentation [remove everything except region around prev_center]
    if (segment && false)
    {
        if(debug)
        {
            std::cout<<"[PoseEstimation::detectionPose] Starting Region Segmentation"<<std::endl;
        }
        
        Eigen::Affine3f t;
        t.matrix() = inital_guess;
        prev_center = t.translation();
        cloud = segmentation::segRegion(cloud, prev_center, segmentation_region_box_size);
        
        if(debug)
        {
            std::cout<<"[PoseEstimation::detectionPose] Region Segmentation Completed: "<< cloud->size()<<" points in cloud"<<std::endl;
        }
    } 
    if(debug)
    {
        std::cout<<"[PoseEstimation::detectionPose] Segementation Completed"<<std::endl;
        std::cout<<"[PoseEstimation::detectionPose] Starting Normal Computation"<<std::endl;

    }
    // Compute scene Normals
    scene_cloud = utils::computeCloudNormals(cloud, compute_normals_radius);
    
    if(debug)
    {
        std::cout<<"[PoseEstimation::detectionPose] Normal Computation Completetd"<<std::endl;
        std::cout<<"[PoseEstimation::detectionPose] Starting Descriptor Computation"<<std::endl;
    }
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features (new  pcl::PointCloud<pcl::FPFHSignature33>);
    scene_features = descriptor::computeDescriptors(scene_cloud, descriptor_scene_radius);
    
    if(debug)
    {
        std::cout<<"[PoseEstimation::detectionPose] Descriptor Computation Completed"<<std::endl;
        std::cout<<"[PoseEstimation::detectionPose] Starting Ransac Alignment"<<std::endl;
    }
    // Alignment
    auto [ransac_transformation, ransac_fitness_score] = alignment::SampleConsensusPrerejective(model_cloud, model_features, scene_cloud, scene_features,
                                                                                                alignment_max_iterations,
                                                                                                alignment_num_samples,
                                                                                                alignment_correspondence_randomness,
                                                                                                alignment_similarity_threshold,
                                                                                                alignment_max_correspondence_distance,
                                                                                                alignment_inlier_fraction);
    if(debug)
    {
        std::cout<<"[PoseEstimation::detectionPose] Ransac Alignment Completed"<<std::endl;
    }
    
    if(ransac_transformation.isIdentity(0))
    {
        std::cout<<"[PoseEstimation::detectionPose] Error Detection: No ransac corresponsdences found, relax alignment paramenters"<<std::endl;
    }
    
    if(debug)
    {
        std::cout<<"[PoseEstimation::detectionPose] Starting ICP"<<std::endl;
    }
    auto[icp_transformation, icp_fitness_score] = alignment::runICP( model_cloud, scene_cloud, ransac_transformation, model_kd_tree,
                                                                     icp_max_iterations, icp_transformation_eps,
                                                                     icp_max_correspondence_distance, icp_fitness_eps,
                                                                     icp_outlier_threshold);

    if(debug)
    {
        std::cout<<"[PoseEstimation::detectionPose] ICP Completed"<<std::endl;
    }
    Eigen::Matrix4f final_transformation =  icp_transformation * ransac_transformation;
    if(debug)
    {
        std::cout<<"[PoseEstimation::detectionPose] Detection Completed"<<std::endl;
    }
    return {final_transformation, icp_fitness_score, cloud};
}

std::tuple<Eigen::Matrix4f, bool> PoseEstimation::compareResults(Eigen::Matrix4f tracking_pose, 
                                                                 float tracking_score, 
                                                                 Eigen::Matrix4f detection_pose, 
                                                                 float detection_score,
                                                                 Eigen::Matrix4f prev_kalman_pose,
                                                                 float time,
                                                                 float speed)
{   
    if(debug)
    {
        std::cout<<"[PoseEstimation::compareResults] Starting Comparison/Sanity checks"<<std::endl;
    }
    
    if(tracking_score <= detection_score)
    {   
        if(debug)
        {
            std::cout<<"[PoseEstimation::compareResults] Not Correcting: score too poor"<<std::endl;
            std::cout<<"[PoseEstimation::compareResults] Ending Comparison/Sanity checks"<<std::endl;
        }
        return {tracking_pose, false};
    }

    Eigen::Affine3f detection_transformation;
    Eigen::Affine3f kalman_transformation;
    Eigen::Vector3f detection_translation, kalman_translation, distance;
    
    detection_transformation.matrix() = detection_pose;
    kalman_transformation.matrix() = prev_kalman_pose;
    detection_translation = detection_transformation.translation();
    kalman_translation = kalman_transformation.translation();
    distance = detection_translation- kalman_translation;

    std::cout << "[PoseEstimation::compareResults] Distance: " << distance.norm() << " Speed: " << speed << " Time: " << time << " Speed * Time: " << speed * time << std::endl;
    if(distance.norm() > speed * time)
    {
        if(debug)
        {
            std::cout<<"[PoseEstimation::compareResults] Not Correcting: distance too far"<<std::endl;
            std::cout<<"[PoseEstimation::compareResults] Ending Comparison/Sanity checks"<<std::endl;
        }
        return {tracking_pose, 0};
    }
    else
    {
        if(debug)
        {
            std::cout<<"[PoseEstimation::compareResults] Correcting"<<std::endl;
            std::cout<<"[PoseEstimation::compareResults] Ending Comparison/Sanity checks"<<std::endl;
        }
       
        return {detection_pose,1};
    }  

}

void PoseEstimation::visualizeTransformation(Eigen::Matrix4f transformation, pcl::PointCloud<pcl::PointNormal>::Ptr scene_cloud)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr transformed_model(new pcl::PointCloud<pcl::PointNormal>);
    pcl::transformPointCloudWithNormals(*model_cloud, *transformed_model, transformation);

    pcl::visualization::PCLVisualizer viewer("Alignment");
    
    viewer.addPointCloud(scene_cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(scene_cloud, 255, 255, 255), "scene_cloud");
    viewer.addPointCloud(transformed_model, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(transformed_model, 0, 0, 255), "transformed_model");
    viewer.addPointCloud(model_cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(model_cloud, 255, 0, 0), "model_cloud");
   
    viewer.spin();
    viewer.close();
}

void PoseEstimation::visualizeAllTransformation(std:: string csv_path, std:: string pcd_path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr trajectory_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(pcd_path, *scene_cloud);
    Eigen::MatrixXd points_mat = CSVParser::readPoseFromCSV<Eigen::MatrixXd>(csv_path);
    double x,y,z;

    for(int i=0; i<points_mat.rows();i++)
    {
        x = points_mat.coeff(i,0);
        y = points_mat.coeff(i,1);
        z = points_mat.coeff(i,2);
        pcl::PointXYZ temp_point(x,y,z);
        trajectory_cloud->push_back(temp_point);
    }
   
    pcl::visualization::PCLVisualizer viewer("Alignment");
    viewer.addPointCloud(scene_cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(scene_cloud, 255, 255, 255), "scene_cloud");
    viewer.addPointCloud(trajectory_cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(trajectory_cloud, 255, 0, 0), "trajectory_cloud");
    viewer.addPolygon<pcl::PointXYZ>(trajectory_cloud, 0,0,255);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "trajectory_cloud");
    viewer.spin();
    viewer.close();
}
