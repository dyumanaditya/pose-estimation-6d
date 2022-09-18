/**
 * @file utils.h
 * @authors Arjun Datta (arjunvirdatta@gmail.com), Dyuman Aditya (dyuman.aditya@gmail.com)
 * @brief Header file for utils with compute normals and obj to cloud functions
 * @version 1.0
 * @date 2021-07-10
 * 
 * @copyright Copyright (c) 2021 (we need licence)
 * 
 */


#ifndef UTILS_H
#define UTILS_H


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



namespace utils
{
    /** 
     * @brief Generate a point cloud from a .obj file
     * 
     * @param obj_file_path Path to .obj file which is to be converted into a point cloud
     * 
     * @return pcl::PointCloud<pcl::PointNormal>::Ptr Pointer to a point cloud
     */
    pcl::PointCloud<pcl::PointNormal>::Ptr objToCloud(std::string obj_file);

    /** 
     * @brief Compute normals of a point cloud
     * 
     * @param cloud Point cloud for which normals are to be calculated
     * @param radius Radius which determines which enighboring points are used to compute normals
     * 
     * @return pcl::PointCloud<pcl::PointNormal>::Ptr Pointer to a point cloud with normals
     */
    pcl::PointCloud<pcl::PointNormal>::Ptr computeCloudNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, float radius);
    
}; // namespace utils


#endif