/**
 * @file descriptor.h
 * @authors Arjun Datta (arjunvirdatta@gmail.com), Dyuman Aditya (dyuman.aditya@gmail.com)
 * @brief Header file for descriptor computation. Fast Point Feature Histogram (FPFH)
 * @version 1.0
 * @date 2021-07-10
 * 
 * @copyright Copyright (c) 2021 (we need licence)
 * 
 */


#ifndef DESCRIPTOR_H
#define DESCRIPTOR_H

#include <pcl/features/fpfh_omp.h>



/**
 * @brief Namespace containing descriptor computing functions
 * 
 */
namespace descriptor
{
    /** 
     * @brief Compute descriptors for a point cloud
     * 
     * @param cloud Pointer to the cloud for which descriptors are to be calculated
     * @param radius Radius within which all points to be taken into account
     * 
     * @return pcl::PointCloud<pcl::FPFHSignature33>::Ptr Pointer to the features/descriptors of the point cloud
     */
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeDescriptors(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, float radius);

}; // namespace descriptor


#endif