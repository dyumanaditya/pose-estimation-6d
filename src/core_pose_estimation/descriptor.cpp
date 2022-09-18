/**
 * @file descriptor.cpp
 * @authors Arjun Datta (arjunvirdatta@gmail.com), Dyuman Aditya (dyuman.aditya@gmail.com)
 * @brief Source file for Fast point feature histogram descriptors 
 * @version 1.0
 * @date 2021-07-10
 * 
 * @copyright Copyright (c) 2021 (we need licence)
 * 
 */


#include "descriptor.h"

#include <pcl/filters/uniform_sampling.h>



pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptor::computeDescriptors(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, 
                                                                          float radius)
{
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features (new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fpfh_est;
    
    fpfh_est.setRadiusSearch(radius);
    fpfh_est.setInputCloud(cloud);
    fpfh_est.setInputNormals(cloud);
    fpfh_est.compute(*features);

    return features;
}